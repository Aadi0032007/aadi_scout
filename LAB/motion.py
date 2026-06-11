# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 20:04:03 2026

@author: Aadi
"""
from __future__ import annotations


"""
Motion: UDP commands → /cmd_vel via ROS2.

This is the ONLY rclpy user in the system. Everything else uses direct
hardware access. The orchestrator calls rclpy.init() once at startup and
shares the global context with this controller.

Behavior:
    - Publishes geometry_msgs/Twist to /cmd_vel at motion_publish_hz.
    - Subscribes to /cmd_vel to echo back what was actually published,
      regardless of source (teleop or inference policy). This is the
      value exposed to the recorder via published_state().
    - Watchdog: if no command arrives within motion_watchdog_sec, output zero.
    - robot_lock=True → output zero regardless of incoming commands.
    - brake=True       → output zero regardless of incoming commands.
    - ang_z is multiplied by ang_z_scale (default 0.20) so turning feels
      proportional to forward speed. Matches the original.
    - Publishes 3× zero on stop() for safety.

The orchestrator passes parsed values into command() — this controller
does no UDP work and doesn't know about source arbitration.

Recording note
--------------
Do NOT pass motion.state to the recorder. It returns raw pre-scaling,
pre-gate values from the last UDP packet, which:
    - has ang_z inflated 5× (ang_z_scale not applied)
    - ignores watchdog / lock / brake zeros
    - returns stale data when there is no teleoperator (inference mode)

Pass motion.published_state instead. It reads whatever was last seen on
/cmd_vel, so it is always correct whether the source is a human operator
or an inference policy.
"""

import threading
import time
from typing import Optional

from .common import log


class MotionController:
    def __init__(
        self,
        topic:            str   = "/cmd_vel",
        publish_hz:       int   = 50,
        watchdog_sec:     float = 0.30,
        ang_z_scale:      float = 0.20,
    ) -> None:
        self._topic         = topic
        self._publish_hz    = max(1, publish_hz)
        self._watchdog      = watchdog_sec
        self._ang_z_scale   = ang_z_scale

        # State (protected by _lock)
        self._lock          = threading.Lock()
        self._lin_x         = 0.0
        self._ang_z         = 0.0
        self._locked        = True   # start locked for safety
        self._braking       = False
        self._last_cmd_t    = 0.0

        # Last values confirmed on /cmd_vel — set by the subscriber echo.
        # This reflects what the robot actually received, whether the source
        # is this controller's publish loop or an inference policy.
        self._last_pub_lin: float = 0.0
        self._last_pub_ang: float = 0.0

        # ROS2 handles
        self._node          = None
        self._pub           = None
        self._sub           = None
        self._executor      = None
        self._executor_thread: Optional[threading.Thread] = None

        # Publisher loop
        self._stop          = threading.Event()
        self._pub_thread    = threading.Thread(target=self._publish_loop, daemon=True, name="motion-pub")

    # ── lifecycle ─────────────────────────────────────────────────────────────

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

            # Subscribe to /cmd_vel so published_state() reflects the actual
            # topic value regardless of whether we published it or an inference
            # policy did. The callback runs on the executor thread; the lock
            # makes the two-field update atomic from the reader's perspective.
            self._sub = self._node.create_subscription(
                Twist,
                self._topic,
                self._on_cmd_vel_echo,
                10,
            )

            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._executor_thread = threading.Thread(
                target=self._executor.spin, daemon=True, name="motion-spin"
            )
            self._executor_thread.start()

            self._pub_thread.start()
            log("motion", f"publishing → {self._topic} @ {self._publish_hz} Hz "
                          f"(watchdog={self._watchdog*1000:.0f}ms, ang_scale={self._ang_z_scale})")
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
        self._sub  = None

    # ── public API ────────────────────────────────────────────────────────────

    def command(self, lin_x: float, ang_z: float, locked: bool, braking: bool) -> None:
        """Update the latest commanded state. Called from the UDP dispatcher."""
        with self._lock:
            self._lin_x      = float(lin_x)
            self._ang_z      = float(ang_z)
            self._locked     = bool(locked)
            self._braking    = bool(braking)
            self._last_cmd_t = time.monotonic()

    def state(self) -> tuple[float, float, bool, bool]:
        """Raw pre-gate state: (lin_x, ang_z, locked, braking).

        This is what the teleoperator commanded before ang_z_scale,
        watchdog, lock, and brake are applied. Used for the stream's
        speed-badge overlay where showing operator intent is appropriate.

        Do NOT pass this to the recorder. Use published_state() there.
        """
        with self._lock:
            return self._lin_x, self._ang_z, self._locked, self._braking

    def published_state(self) -> tuple[float, float]:
        """Return the last (linear_x, angular_z) confirmed on /cmd_vel.

        This is what the robot actually received: post ang_z_scale,
        post watchdog, post lock/brake. It is updated by the /cmd_vel
        subscriber so it works correctly whether the publisher is this
        controller's loop (teleoperation) or an external inference policy.

        Always pass THIS to the recorder, not state().
        """
        with self._lock:
            return self._last_pub_lin, self._last_pub_ang

    # ── /cmd_vel subscriber echo ─────────────────────────────────────────────

    def _on_cmd_vel_echo(self, msg) -> None:
        """Subscriber callback: capture whatever lands on /cmd_vel.

        Runs on the executor (motion-spin) thread. The lock makes the
        two-field write atomic from published_state()'s perspective.
        The critical section is ~50 ns — no contention risk.
        """
        with self._lock:
            self._last_pub_lin = msg.linear.x
            self._last_pub_ang = msg.angular.z

    # ── publisher loop ────────────────────────────────────────────────────────

    def _publish_loop(self) -> None:
        interval = 1.0 / self._publish_hz
        while not self._stop.is_set():
            lin, ang = self._compute_output()
            self._send_twist(lin, ang)
            self._stop.wait(timeout=interval)

    def _compute_output(self) -> tuple[float, float]:
        """Apply all safety gates and return (lin_x, ang_z) to publish this tick."""
        with self._lock:
            now          = time.monotonic()
            watchdog_ok  = (now - self._last_cmd_t) < self._watchdog
            locked       = self._locked
            braking      = self._braking
            lin_x        = self._lin_x
            ang_z        = self._ang_z * self._ang_z_scale

        if not watchdog_ok or locked or braking:
            return 0.0, 0.0
        return lin_x, ang_z

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