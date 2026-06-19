# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 20:04:03 2026

@author: Aadi
"""
from __future__ import annotations


"""
Motion: UDP commands → /cmd_vel via ROS2, with human/AI source arbitration.

This is still the ONLY rclpy user in the system. The orchestrator calls
rclpy.init() once at startup and shares the global context with this
controller. Publishes geometry_msgs/Twist to /cmd_vel at publish_hz.

What's new
----------
Two command sources are arbitrated internally with HARD HUMAN PRIORITY:

  • Human commands always win during a "handback" window that opens on
    any meaningful human input and stays open for human_handback_sec
    (default 2.0s) past the last meaningful sample.
  • AI commands are only honored when BOTH:
        1. set_ai_enabled(True) was called (explicit human consent), AND
        2. the handback window is closed (human is idle).
  • Human BRAKE is a hard latch — it disables AI until an explicit
    set_ai_enabled(True) re-enables it.
  • Human is ALWAYS authoritative for lock and brake. AI cannot unlock
    and cannot un-brake.

Existing gates (watchdog, lock, brake, ang_z_scale, 3× zero on stop)
are unchanged in behavior — they now apply to whichever source the
arbiter picked this tick.

Public API additions (back-compatible — origin defaults to "human"):
    command(lin_x, ang_z, locked, braking, origin="human")
    set_ai_enabled(on: bool)
    ai_enabled() -> bool
    human_in_control() -> bool

Unchanged:
    state()           → human pre-gate intent (for overlay/badge)
    published_state() → post-gate values actually sent (for recorder)
    start() / stop()

Recording note (unchanged)
--------------------------
Do NOT pass motion.state to the recorder. Pass motion.published_state.
published_state() reflects whichever source the arbiter selected,
post-scale and post-gate, so the recorder always sees what /cmd_vel
actually received.
"""

import threading
import time
from typing import Optional

from .common import log


class MotionController:
    def __init__(
        self,
        topic:               str   = "/cmd_vel",
        publish_hz:          int   = 50,
        watchdog_sec:        float = 0.30,
        ang_z_scale:         float = 0.20,
        # ── NEW: human-priority arbiter knobs ────────────────────────────
        human_handback_sec:  float = 2.0,
        human_idle_deadband: float = 0.05,
    ) -> None:
        self._topic         = topic
        self._publish_hz    = max(1, publish_hz)
        self._watchdog      = watchdog_sec
        self._ang_z_scale   = ang_z_scale

        # ── State (protected by _lock) ───────────────────────────────────
        self._lock = threading.Lock()

        # Per-origin latest command: (lin_x, ang_z, locked, braking, t_monotonic)
        # Human starts LOCKED (safety). AI starts inert AND gated off.
        self._latest_human: tuple[float, float, bool, bool, float] = (
            0.0, 0.0, True, False, 0.0
        )
        self._latest_ai:    tuple[float, float, bool, bool, float] = (
            0.0, 0.0, False, False, 0.0
        )

        # AI control gate. Latched off; flipped on only by an explicit
        # set_ai_enabled(True) call from teleop on the human's enable chord.
        # Flipped back off by human brake or set_ai_enabled(False).
        self._ai_enabled = False

        # Handback window. While now < _human_active_until, human is "in
        # control" even if AI is publishing. Extended on every human
        # command above the idle deadband. After it expires AND AI is
        # enabled, AI resumes driving.
        self._human_active_until = 0.0
        self._human_handback_sec = float(human_handback_sec)
        self._human_idle_db      = float(human_idle_deadband)

        # Last values actually published to /cmd_vel — for recorder.
        # Updated synchronously inside _publish_loop, same as before.
        self._last_pub_lin: float = 0.0
        self._last_pub_ang: float = 0.0

        # ROS2 handles (unchanged from previous version)
        self._node          = None
        self._pub           = None
        self._executor      = None
        self._executor_thread: Optional[threading.Thread] = None

        # Publisher loop
        self._stop       = threading.Event()
        self._pub_thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="motion-pub"
        )

    # ── lifecycle ─────────────────────────────────────────────────────────

    def start(self) -> None:
        """Create the ROS2 node and start the publish loop.

        rclpy.init() must already have been called by the orchestrator.
        """
        try:
            import rclpy   # type: ignore
            from rclpy.executors import SingleThreadedExecutor   # type: ignore
            from geometry_msgs.msg import Twist                  # type: ignore

            if not rclpy.ok():
                log("motion", "rclpy not initialized — orchestrator must call rclpy.init() first")
                return

            self._node = rclpy.create_node("lab_motion")
            self._pub  = self._node.create_publisher(Twist, self._topic, 10)

            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._executor_thread = threading.Thread(
                target=self._executor.spin, daemon=True, name="motion-spin"
            )
            self._executor_thread.start()

            self._pub_thread.start()
            log(
                "motion",
                f"publishing → {self._topic} @ {self._publish_hz} Hz "
                f"(watchdog={self._watchdog*1000:.0f}ms, ang_scale={self._ang_z_scale}, "
                f"handback={self._human_handback_sec}s, idle_db={self._human_idle_db})"
            )
        except ImportError:
            log("motion", "rclpy not installed — motion disabled")
        except Exception as exc:
            log("motion", f"start failed: {exc}")

    def stop(self) -> None:
        self._stop.set()
        try:
            self._pub_thread.join(timeout=1.0)
        except Exception:
            pass

        # Publish 3× zero for safety on shutdown
        for _ in range(3):
            self._send_twist(0.0, 0.0)
            time.sleep(0.02)

        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        self._node = None
        self._pub  = None

    # ── public API ────────────────────────────────────────────────────────

    def command(
        self,
        lin_x:   float,
        ang_z:   float,
        locked:  bool,
        braking: bool,
        origin:  str = "human",
    ) -> None:
        """Update the latest command for one source.

        origin = "human" (default, back-compat) or "ai".

        Human path also:
          • extends the handback window on meaningful input,
          • latches AI off on brake.
        AI path only updates the latest-AI tuple — whether it actually
        drives /cmd_vel is decided each publish tick.
        """
        now = time.monotonic()
        with self._lock:
            if origin == "ai":
                self._latest_ai = (float(lin_x), float(ang_z),
                                   bool(locked), bool(braking), now)
                return

            # Human path
            self._latest_human = (float(lin_x), float(ang_z),
                                  bool(locked), bool(braking), now)

            meaningful = (
                abs(lin_x) > self._human_idle_db
                or abs(ang_z) > self._human_idle_db
                or bool(braking)
            )
            if meaningful:
                self._human_active_until = now + self._human_handback_sec

            if braking:
                # HARD LATCH: human brake disables AI until explicitly
                # re-enabled.
                if self._ai_enabled:
                    log("motion", "AI disabled by human brake")
                self._ai_enabled = False

    def set_ai_enabled(self, on: bool) -> None:
        """Toggle the AI control gate. Called by teleop on the human's
        explicit enable chord (and any explicit disable path)."""
        with self._lock:
            prev = self._ai_enabled
            self._ai_enabled = bool(on)
            if not on:
                # Wipe stale AI state so a fresh enable starts clean.
                self._latest_ai = (0.0, 0.0, False, False, 0.0)
            if prev != self._ai_enabled:
                log("motion", f"ai_enabled -> {self._ai_enabled}")

    def state(self) -> tuple[float, float, bool, bool]:
        """Human operator's pre-gate intent: (lin_x, ang_z, locked, braking).

        Used by the stream's speed-badge overlay. Returns the most recent
        HUMAN command — not whatever is currently driving the robot — so
        the badge reflects operator intent.

        Do NOT pass this to the recorder. Use published_state().
        """
        with self._lock:
            lin, ang, locked, braking, _t = self._latest_human
            return lin, ang, locked, braking

    def published_state(self) -> tuple[float, float]:
        """Last (lin_x, ang_z) actually sent to /cmd_vel — for the recorder.

        Reflects whichever source was selected by the arbiter and after
        ang_z_scale, watchdog, lock, and brake gates. Pass THIS to the
        recorder, not state().
        """
        with self._lock:
            return self._last_pub_lin, self._last_pub_ang

    def ai_enabled(self) -> bool:
        with self._lock:
            return self._ai_enabled

    def human_in_control(self) -> bool:
        """True iff human is currently driving (handback open or AI gated off)."""
        with self._lock:
            return (time.monotonic() < self._human_active_until) or (not self._ai_enabled)

    # ── publisher loop ────────────────────────────────────────────────────

    def _publish_loop(self) -> None:
        interval = 1.0 / self._publish_hz
        while not self._stop.is_set():
            lin, ang = self._compute_output()
            self._send_twist(lin, ang)

            # Store synchronously — published_state() returns this to the recorder.
            with self._lock:
                self._last_pub_lin = lin
                self._last_pub_ang = ang

            self._stop.wait(timeout=interval)

    def _compute_output(self) -> tuple[float, float]:
        """Pick a source (human vs AI), then apply universal gates.

        Returns the (lin_x, ang_z) to publish THIS tick — ang_z already
        scaled by ang_z_scale.
        """
        with self._lock:
            now = time.monotonic()
            h_lin, h_ang, h_locked, h_brake, h_t = self._latest_human
            a_lin, a_ang, _a_lk, _a_br,  a_t    = self._latest_ai

            ai_enabled      = self._ai_enabled
            handback_active = now < self._human_active_until

            # Source selection. Human wins during handback OR when AI is gated off.
            human_in_control = handback_active or (not ai_enabled)

            if human_in_control:
                src_lin, src_ang, src_t = h_lin, h_ang, h_t
            else:
                src_lin, src_ang, src_t = a_lin, a_ang, a_t

            # Per-source watchdog — if the selected source hasn't published
            # within _watchdog seconds, zero. We do NOT fall back to the
            # other source: the selected source going stale is suspicious
            # and silence is the safe default.
            watchdog_ok = (now - src_t) < self._watchdog

            # Human is authoritative for lock + brake regardless of who drives.
            # AI cannot unlock and cannot un-brake.
            locked  = h_locked
            braking = h_brake

        if not watchdog_ok or locked or braking:
            return 0.0, 0.0
        return src_lin, src_ang * self._ang_z_scale

    def _send_twist(self, lin: float, ang: float) -> None:
        if self._pub is None:
            return
        try:
            from geometry_msgs.msg import Twist   # type: ignore
            t = Twist()
            t.linear.x  = float(lin)
            t.angular.z = float(ang)
            self._pub.publish(t)
        except Exception:
            pass