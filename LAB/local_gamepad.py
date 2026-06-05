# -*- coding: utf-8 -*-
"""
Created on Fri Jun  5 12:00:00 2026

@author: Aadi
"""
from __future__ import annotations

"""
Local gamepad — pygame-based, mirrors the operator-side script exactly.

Replaces the old `LocalDongleListener` (which only read steering + d-pad via
evdev). This version uses pygame so the mapping tables, controller selection,
and state machine are literally copy-paste from
`prod_revopilot_udp_highlowfreq_gamepad_cmds_win_lin.py` — keeping the
two code paths in lockstep.

What's mirrored from the operator:
    - 8BitDo Ultimate / Ultimate 2 mapping profiles (auto-selected by name+OS)
    - Steering with deadzone + expo + speed-aware yaw limit
    - Cruise level cycling (-1.0 … +1.0)
    - Speed-level cycling on repeat A→B unlock (1=slow, 2=medium, 3=fast)
    - A→B / B→A lock sequences (2 s window) with camera-state restore
    - Axis 3 indicators (left / right turn signals)
    - Axis 4 sound shortcuts (single tap = speech, double tap = music)
    - Lights ON / Lights OFF dedicated buttons
    - X / Y camera cycling
    - Lift axes
    - Identical payload field names and rounding

What's different from the operator:
    - Doesn't open a UDP socket. Calls the orchestrator's three dispatchers
      (`on_motion`, `on_events`, `on_tts`) directly.
    - Packets carry `"_local": True` so SourceArbiter routes them as the
      local source.

Activity gating (added to fix stream/drive interruption):
    The 8BitDo USB receiver dongle is detected as a joystick by pygame even
    when the controller itself is OFF. Without gating, this thread would
    emit motion packets at 50 Hz the moment the receiver is plugged in,
    and — because local priority is lower than remote — would stomp the
    remote source continuously with `robot_lock=True`, killing the stream
    and recording.

    Two guards now apply:
        1. Motion dispatch is gated on real user activity (any button held,
           any axis past its threshold, any cruise/lock sequence in flight),
           plus a short grace window after the last activity. When idle,
           NO packet is dispatched — the arbiter never sees "local" and
           the remote source drives normally.
        2. The `robot_lock` field is omitted from the payload until the
           user has actually performed a lock sequence (A→B or B→A) on
           this gamepad. Until then, the orchestrator's parse_lock_state
           keeps the previous lock state instead of forcing it to True.
"""

import math
import os
import platform
import threading
import time
from typing import Callable, Optional

# Suppress SDL display/audio init on headless robots. Must run BEFORE pygame
# is imported anywhere in the process.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

from .common import log


# ── Tunables — mirror operator-side constants ─────────────────────────────────

SEND_HZ = 50
JOYSTICK_RETRY_SEC = 1.0
AXIS_ACTION_THRESHOLD_COUNTS = 30000

AXIS4_NEG_MESSAGE = "Hellow how are you today?"
AXIS4_POS_MESSAGE = "please let me go!"
TALK_DURATION_SEC = 7.0
AXIS4_MULTI_TAP_WINDOW_SEC = 1.0
MUSIC_TRACK_ID = 1
MUSIC_TALK_DURATION_SEC = 60.0
AUDIO_FULL_VOLUME_PCT = 100

# Steering
MAX_YAW_MOVING = 2.0
MAX_YAW_INPLACE = 3.5
STEER_DEADZONE = 0.1
STEER_EXPO = 0.8
STEER_GAIN = 1.0

# Pedal & cruise (pedal-driven lin_x is disabled on operator side too)
BRAKE_THRESHOLD = 0.2
CRUISE_LEVELS = [-1.0, -0.6, -0.4, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2, 0.4, 0.6, 1.0]
PEDAL_DEADBAND = 0.05

LIFT_MIN_CMD = 50
LIFT_MAX_CMD = 255
LIFT_AXIS_DEADBAND = 0.02

LOCK_SEQUENCE_TIMEOUT = 2.0
MAX_SPEED_INITIAL = 1.0
SWAP_XY_BUTTONS = False

# ── Activity-gate tunables ────────────────────────────────────────────────────
# How long after the last detected activity we keep dispatching motion packets.
# Prevents a momentary button-up or steering centring from bouncing arbitration
# back to remote mid-action.
LOCAL_ACTIVE_GRACE_SEC = 1.5
# Head-axis activation threshold (matches the PTZ direction threshold used
# later in the loop, so we only count it as "activity" once it would actually
# command the camera).
HEAD_ACTIVATION_THRESHOLD = 0.5


# ── Gamepad profiles — identical to operator script ──────────────────────────

GAMEPAD_MAPPINGS = {
    "8bitdo_ultimate_wireless_pc": {
        "axis_steer": 0,
        "axis_sound": 3,
        "axis_signal": 4,
        "axis_head_lr": 6,
        "axis_head_ud": 7,
        "axis_lift_pos": 4,
        "axis_lift_neg": 5,
        "btn_a": 0,
        "btn_b": 1,
        "btn_x": 3,
        "btn_y": 4,
        "btn_cruise_down": 6,
        "btn_cruise_up": 7,
        "btn_lights_on": 11,
        "btn_lights_off": 10,
    },
    "8bitdo_ultimate2_wireless": {
        "axis_steer": 0,
        "axis_signal": 3,
        "axis_sound": 4,
        "axis_head_lr": 6,
        "axis_head_ud": 7,
        "axis_lift_pos": 5,
        "axis_lift_neg": 2,
        "btn_a": 0,
        "btn_b": 1,
        "btn_x": 2,
        "btn_y": 3,
        "btn_cruise_down": 4,
        "btn_cruise_up": 5,
        "btn_lights_on": 7,
        "btn_lights_off": 6,
    },
    # Windows profile kept for completeness even though the robot is Linux —
    # makes it trivial to test the same module on a Windows dev box.
    "8bitdo_ultimate2_wireless_windows": {
        "axis_steer": 0,
        "axis_signal": 2,
        "axis_sound": 3,
        "axis_head_lr": 6,
        "axis_head_ud": 7,
        "axis_lift_pos": 5,
        "axis_lift_neg": 4,
        "btn_a": 0,
        "btn_b": 1,
        "btn_x": 2,
        "btn_y": 3,
        "btn_cruise_down": 4,
        "btn_cruise_up": 5,
        "btn_lights_on": 7,
        "btn_lights_off": 6,
    },
}

BUTTON_NUM_MAP = {
    "btn_a": 1,
    "btn_b": 2,
    "btn_x": 3,
    "btn_y": 4,
    "btn_cruise_down": 5,
    "btn_cruise_up": 6,
    "btn_lights_off": 7,
    "btn_lights_on": 8,
}

DEFAULT_MAPPING_KEY = "8bitdo_ultimate_wireless_pc"


# ── Math helpers — identical to operator script ──────────────────────────────

def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _deadzone(x: float, dz: float) -> float:
    if abs(x) <= dz:
        return 0.0
    return math.copysign((abs(x) - dz) / (1.0 - dz), x)


def _expo(x: float, expo: float) -> float:
    return (1.0 - expo) * x + expo * (x ** 3)


def _lift_axis_to_cmd(axis_value: float) -> int:
    """Map trigger axis [-1..+1] (rest=-1, pressed=+1) to command [0 or 50..255]."""
    press = _clamp((axis_value + 1.0) * 0.5, 0.0, 1.0)
    if press <= LIFT_AXIS_DEADBAND:
        return 0
    return int(round(LIFT_MIN_CMD + press * (LIFT_MAX_CMD - LIFT_MIN_CMD)))


# ── The thread ───────────────────────────────────────────────────────────────

class LocalGamepad(threading.Thread):
    """Reads a locally-plugged 8BitDo controller and feeds the orchestrator.

    Construction:
        on_motion / on_events / on_tts are the three dispatcher callbacks
        already defined in teleop.py main(). Same signatures as the UDP
        listener uses, so packets are formatted identically to remote ones.

    The local source emits `"_local": True` in every packet so the arbiter
    routes them under the "local" priority slot.
    """

    def __init__(
        self,
        on_motion: Callable[[dict, tuple, int], None],
        on_events: Callable[[dict, tuple, int], None],
        on_tts:    Callable[[dict, tuple, int], None],
        initial_robot_lock: bool = True,
        priority_value:     int  = 100,
    ) -> None:
        super().__init__(daemon=True, name="local-gamepad")
        self._on_motion = on_motion
        self._on_events = on_events
        self._on_tts    = on_tts
        self._priority  = priority_value
        self._init_lock = initial_robot_lock
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    # ── thread main ──────────────────────────────────────────────────────────

    def run(self) -> None:
        try:
            import pygame
        except ImportError:
            log("local_gp", "pygame not installed — local gamepad disabled "
                            "(pip install pygame)")
            return

        try:
            pygame.init()
            pygame.joystick.init()
        except Exception as exc:
            log("local_gp", f"pygame init failed: {exc}")
            return

        # Initial state (mirrors operator main())
        seq = 0
        speech_seq = 0
        period = 1.0 / SEND_HZ
        next_t = time.time()

        cruise_zero_idx = CRUISE_LEVELS.index(0.0)
        cruise_level_idx = cruise_zero_idx

        camera_modes = ["floor", "orbital", "ai_front", "ai_back"]
        camera_index = 1
        camera_mode = camera_modes[camera_index]

        robot_lock = self._init_lock
        max_speed = MAX_SPEED_INITIAL
        speed_level = 1

        lock_seq: list = []
        seq_start_state: Optional[str] = None
        sequence_active = False

        prev_a = prev_b = prev_x = prev_y = 0
        prev_cruise_up = prev_cruise_down = 0
        prev_lights_on = prev_lights_off = 0

        axis3_state = "center"
        axis4_state = "center"
        axis4_neg_taps: list = []
        axis4_pending_speech_deadline: Optional[float] = None

        # ── Activity-gate state ─────────────────────────────────────────────
        # `last_active_t = 0.0` keeps us idle at startup; we only start
        # emitting motion packets once the user actually does something.
        last_active_t = 0.0
        was_engaged = False   # for one-shot log on engagement / release

        # `lock_user_engaged` flips True the first time the user completes
        # an A→B or B→A sequence on this gamepad. Until then, we omit
        # `robot_lock` from the payload so we don't override remote.
        lock_user_engaged = False

        js = self._wait_for_joystick(pygame)
        if js is None:
            return
        gamepad = self._select_mapping(js)

        while not self._stop.is_set():
            now = time.time()
            if now < next_t:
                # Cap sleep so stop() is responsive
                time.sleep(min(0.05, next_t - now))
                continue
            next_t = now + period

            # ── reconnect path ───────────────────────────────────────────────
            if pygame.joystick.get_count() == 0:
                log("local_gp", "joystick disconnected, waiting for reconnect…")
                js = self._wait_for_joystick(pygame)
                if js is None:
                    return
                gamepad = self._select_mapping(js)
                next_t = time.time()
                continue

            try:
                pygame.event.pump()
            except pygame.error:
                log("local_gp", "joystick event error, reconnecting…")
                js = self._wait_for_joystick(pygame)
                if js is None:
                    return
                gamepad = self._select_mapping(js)
                next_t = time.time()
                continue

            # ── raw reads ────────────────────────────────────────────────────
            raw_steer   = -self._read_axis(pygame, js, gamepad["axis_steer"])
            head_lr     =  self._read_axis(pygame, js, gamepad["axis_head_lr"])
            head_ud     =  self._read_axis(pygame, js, gamepad["axis_head_ud"])
            signal_axis =  self._read_axis_counts(pygame, js, gamepad["axis_signal"])
            sound_axis  =  self._read_axis_counts(pygame, js, gamepad["axis_sound"])

            # Hat fallback if controller exposes d-pad as hat rather than axes
            if abs(head_lr) < 0.01 and abs(head_ud) < 0.01 and js.get_numhats() > 0:
                hx, hy = js.get_hat(0)
                head_lr = float(hx)
                head_ud = float(-hy)

            # ── axis 3: turn-signal indicators ───────────────────────────────
            if signal_axis < -AXIS_ACTION_THRESHOLD_COUNTS:
                new_axis3 = "left"
            elif signal_axis > AXIS_ACTION_THRESHOLD_COUNTS:
                new_axis3 = "right"
            else:
                new_axis3 = "center"

            if new_axis3 != axis3_state:
                if new_axis3 == "left":
                    self._emit_event({"event": "signals", "left": True,  "right": False})
                elif new_axis3 == "right":
                    self._emit_event({"event": "signals", "left": False, "right": True})
                axis3_state = new_axis3

            # ── axis 4: sound / music shortcuts ──────────────────────────────
            axis4_now = time.time()

            # Deferred single-tap → speech (operator sends after multi-tap window)
            if (axis4_pending_speech_deadline is not None
                    and axis4_now >= axis4_pending_speech_deadline):
                self._emit_event({"event": "audio", "volume_pct": AUDIO_FULL_VOLUME_PCT})
                self._emit_event({"event": "talk",  "duration": TALK_DURATION_SEC})
                self._emit_tts(AXIS4_NEG_MESSAGE, speech_seq)
                speech_seq += 1
                axis4_pending_speech_deadline = None
                axis4_neg_taps = []

            if sound_axis < -AXIS_ACTION_THRESHOLD_COUNTS:
                new_axis4 = "neg"
            elif sound_axis > AXIS_ACTION_THRESHOLD_COUNTS:
                new_axis4 = "pos"
            else:
                new_axis4 = "center"

            if new_axis4 != axis4_state:
                if new_axis4 == "neg":
                    axis4_neg_taps = [t for t in axis4_neg_taps
                                      if axis4_now - t <= AXIS4_MULTI_TAP_WINDOW_SEC]
                    axis4_neg_taps.append(axis4_now)
                    if len(axis4_neg_taps) >= 2:
                        # Double tap → music + extended talk blink
                        self._emit_event({"event": "music", "action": "play",
                                          "track": MUSIC_TRACK_ID})
                        self._emit_event({"event": "talk",
                                          "duration": MUSIC_TALK_DURATION_SEC})
                        axis4_pending_speech_deadline = None
                        axis4_neg_taps = []
                    else:
                        # Single tap → defer speech until window expires
                        axis4_pending_speech_deadline = axis4_now + AXIS4_MULTI_TAP_WINDOW_SEC
                elif new_axis4 == "pos":
                    self._emit_event({"event": "audio", "volume_pct": AUDIO_FULL_VOLUME_PCT})
                    self._emit_event({"event": "talk",  "duration": TALK_DURATION_SEC})
                    self._emit_tts(AXIS4_POS_MESSAGE, speech_seq)
                    speech_seq += 1
                axis4_state = new_axis4

            # ── pedal-driven lin_x is disabled (operator removed it) ─────────
            pedal_signed = 0.0
            accel = 0.0
            brake = 0.0

            # ── button reads ─────────────────────────────────────────────────
            a   = self._read_btn(js, gamepad["btn_a"])
            b   = self._read_btn(js, gamepad["btn_b"])
            x   = self._read_btn(js, gamepad["btn_x"])
            y   = self._read_btn(js, gamepad["btn_y"])
            cu  = self._read_btn(js, gamepad["btn_cruise_up"])
            cd  = self._read_btn(js, gamepad["btn_cruise_down"])
            lon = self._read_btn(js, gamepad["btn_lights_on"])
            loff = self._read_btn(js, gamepad["btn_lights_off"])

            # Lift
            lift_pos_axis = self._read_axis(pygame, js, gamepad["axis_lift_pos"])
            lift_neg_axis = self._read_axis(pygame, js, gamepad["axis_lift_neg"])
            lp = _lift_axis_to_cmd(lift_pos_axis)
            ln = _lift_axis_to_cmd(lift_neg_axis)
            if lp > ln:
                lift = lp
            elif ln > lp:
                lift = -ln
            else:
                lift = 0

            if SWAP_XY_BUTTONS:
                y, x = x, y

            # Edges
            a_edge   = a   and not prev_a
            b_edge   = b   and not prev_b
            x_edge   = x   and not prev_x
            y_edge   = y   and not prev_y
            lon_edge  = lon  and not prev_lights_on
            loff_edge = loff and not prev_lights_off

            # ── lights buttons → events ──────────────────────────────────────
            if lon_edge:
                self._emit_event({"event": "lights",
                                  "headlights": True,
                                  "parklights": True,
                                  "strobe": True})
                log("local_gp", "lights ON (button)")
            if loff_edge:
                self._emit_event({"event": "lights",
                                  "headlights": False,
                                  "parklights": False,
                                  "strobe": False})
                log("local_gp", "lights OFF (button)")

            # ── lock-sequence bookkeeping ────────────────────────────────────
            pre_camera_mode = camera_mode
            now_t = time.time()

            if a_edge:
                if not lock_seq and seq_start_state is None:
                    seq_start_state = pre_camera_mode
                lock_seq.append(("A", now_t))
            if b_edge:
                if not lock_seq and seq_start_state is None:
                    seq_start_state = pre_camera_mode
                lock_seq.append(("B", now_t))
            if y_edge:
                if not lock_seq and seq_start_state is None:
                    seq_start_state = pre_camera_mode
                lock_seq.append(("Y", now_t))
            if x_edge:
                if not lock_seq and seq_start_state is None:
                    seq_start_state = pre_camera_mode
                lock_seq.append(("X", now_t))

            prev_a, prev_b, prev_x, prev_y = a, b, x, y
            prev_lights_on, prev_lights_off = lon, loff

            # Trim entries older than the window
            lock_seq = [(k, t) for (k, t) in lock_seq
                        if now_t - t <= LOCK_SEQUENCE_TIMEOUT]

            if not sequence_active and len(lock_seq) >= 2:
                first2 = "".join([k for (k, _) in lock_seq[:2]])
                if first2 in ("AB", "BA"):
                    sequence_active = True

            sequence_matched = False
            if len(lock_seq) >= 2:
                last2 = lock_seq[-2:]
                seq_str = "".join([k for (k, _) in last2])
                span = last2[-1][1] - last2[0][1]
                if span <= LOCK_SEQUENCE_TIMEOUT:
                    if seq_str == "AB":
                        if robot_lock:
                            robot_lock = False
                            max_speed = 1.0
                            speed_level = 1
                            log("local_gp", "robot unlocked (A→B)")
                        else:
                            speed_level += 1
                            if speed_level > 3:
                                speed_level = 1
                            max_speed = float(speed_level)
                            log("local_gp", f"speed cycle → level {speed_level}")
                        lock_seq = []
                        sequence_matched = True
                        lock_user_engaged = True
                    elif seq_str == "BA":
                        robot_lock = True
                        log("local_gp", "robot locked (B→A)")
                        lock_seq = []
                        sequence_matched = True
                        lock_user_engaged = True

            if sequence_matched:
                # Restore camera selection that was active when the sequence started
                camera_mode = seq_start_state if seq_start_state is not None else pre_camera_mode
                seq_start_state = None
                sequence_active = False
            elif not lock_seq and seq_start_state is not None:
                seq_start_state = None
                sequence_active = False

            # ── camera cycling (X/Y) — only when no lock sequence in flight ──
            if not sequence_matched and not sequence_active:
                if x_edge:
                    camera_index = (camera_index + 1) % len(camera_modes)
                    camera_mode = camera_modes[camera_index]
                if y_edge:
                    camera_index = (camera_index - 1) % len(camera_modes)
                    camera_mode = camera_modes[camera_index]

            # ── derive payload `button` field ────────────────────────────────
            current_button = 0
            if a:        current_button = BUTTON_NUM_MAP["btn_a"]
            elif b:      current_button = BUTTON_NUM_MAP["btn_b"]
            elif x:      current_button = BUTTON_NUM_MAP["btn_x"]
            elif y:      current_button = BUTTON_NUM_MAP["btn_y"]
            elif cd:     current_button = BUTTON_NUM_MAP["btn_cruise_down"]
            elif cu:     current_button = BUTTON_NUM_MAP["btn_cruise_up"]
            elif loff:   current_button = BUTTON_NUM_MAP["btn_lights_off"]
            elif lon:    current_button = BUTTON_NUM_MAP["btn_lights_on"]

            # ── cruise / brake / final lin_x ─────────────────────────────────
            both_cruise_pressed = bool(cu and cd)
            brake_active = (brake > BRAKE_THRESHOLD) or both_cruise_pressed

            pedal_speed = pedal_signed * max_speed
            if abs(pedal_speed) <= PEDAL_DEADBAND:
                pedal_speed = 0.0

            if both_cruise_pressed:
                cruise_level_idx = cruise_zero_idx
            else:
                if cu and not prev_cruise_up:
                    cruise_level_idx = min(cruise_level_idx + 1, len(CRUISE_LEVELS) - 1)
                if cd and not prev_cruise_down:
                    cruise_level_idx = max(cruise_level_idx - 1, 0)

            cruise_speed = CRUISE_LEVELS[cruise_level_idx]
            prev_cruise_up, prev_cruise_down = cu, cd

            cruise_abs_max = min(1.0, max_speed)
            cruise_speed = _clamp(cruise_speed, -cruise_abs_max, cruise_abs_max)

            if brake_active:
                lin_x = 0.0
                cruise_speed = 0.0
                cruise_level_idx = cruise_zero_idx
            elif abs(pedal_speed) > 0.0:
                lin_x = pedal_speed
                cruise_speed = 0.0
                cruise_level_idx = cruise_zero_idx
            else:
                lin_x = cruise_speed

            # ── PTZ head direction ───────────────────────────────────────────
            axis_head_threshold = HEAD_ACTIVATION_THRESHOLD
            head = "center"
            if   head_lr < -axis_head_threshold: head = "left"
            elif head_lr >  axis_head_threshold: head = "right"
            elif head_ud < -axis_head_threshold: head = "up"
            elif head_ud >  axis_head_threshold: head = "down"

            # ── steering / yaw ───────────────────────────────────────────────
            s = _deadzone(raw_steer, STEER_DEADZONE)
            s = _expo(s, STEER_EXPO)
            s *= STEER_GAIN
            s = _clamp(s, -1.0, 1.0)

            speed_frac = min(abs(lin_x) / max_speed, 1.0) if max_speed > 0 else 0.0
            yaw_limit = MAX_YAW_INPLACE * (1.0 - speed_frac) + MAX_YAW_MOVING * speed_frac
            ang_z = _clamp(s * yaw_limit, -yaw_limit, yaw_limit)
            if both_cruise_pressed:
                ang_z = 0.0

            speed_label = {1: "slow", 2: "medium", 3: "fast"}.get(speed_level, "slow")

            # ── activity gate ────────────────────────────────────────────────
            # Decide if THIS tick counts as user activity. If not (and we're
            # past the grace window), skip dispatch entirely — arbiter never
            # sees "local" and the remote source drives normally.
            steer_active  = abs(raw_steer)   > STEER_DEADZONE
            head_active   = (abs(head_lr) > HEAD_ACTIVATION_THRESHOLD
                             or abs(head_ud) > HEAD_ACTIVATION_THRESHOLD)
            signal_active = abs(signal_axis) > AXIS_ACTION_THRESHOLD_COUNTS
            sound_active  = abs(sound_axis)  > AXIS_ACTION_THRESHOLD_COUNTS
            lift_active   = lift != 0
            btn_active    = bool(a or b or x or y or cu or cd or lon or loff)
            cruise_active = cruise_level_idx != cruise_zero_idx
            seq_in_flight = bool(lock_seq) or sequence_active

            is_active_now = (
                steer_active or head_active or signal_active or sound_active
                or lift_active or btn_active or cruise_active or seq_in_flight
            )

            if is_active_now:
                last_active_t = now

            local_engaged = (last_active_t > 0.0
                             and (now - last_active_t) <= LOCAL_ACTIVE_GRACE_SEC)

            # One-shot logs on transition (helps verify the gate is doing its job).
            if local_engaged and not was_engaged:
                log("local_gp", "ENGAGED — taking over arbitration")
                was_engaged = True
            elif not local_engaged and was_engaged:
                log("local_gp", "released — yielding to remote")
                was_engaged = False

            if not local_engaged:
                # Idle: do NOT dispatch. Remote stays the active source.
                # Advance seq so any log scraping by sequence number still
                # reflects real time, but don't touch on_motion.
                seq += 1
                continue

            # ── build and emit motion packet ─────────────────────────────────
            payload = {
                "seq":        seq,
                "t":          time.time(),
                "lin_x":      round(lin_x,        4),
                "ang_z":      round(ang_z,        4),
                "accel":      round(accel,        3),
                "brake":      round(brake,        3),
                "cruise":     round(cruise_speed, 3),
                "fwd":        True,
                "camera":     camera_mode,
                "head":       head,
                "speed":      speed_label,
                "lift":       lift,
                "priority":   self._priority,
                "button":     current_button,
                "_local":     True,
            }

            # Only assert robot_lock once the user has performed at least one
            # A→B or B→A sequence on THIS gamepad. Until then, omit the field
            # and let parse_lock_state keep the previous (remote) value —
            # otherwise the very first local packet would force the robot
            # back into its init lock state and kill an active remote session.
            if lock_user_engaged:
                payload["robot_lock"] = robot_lock

            seq += 1

            try:
                self._on_motion(payload, ("local", 0), -1)
            except Exception as exc:
                log("local_gp", f"motion dispatch error: {exc}")

        # Loop exit — release pygame
        try:
            pygame.joystick.quit()
            pygame.quit()
        except Exception:
            pass

    # ── pygame helpers ───────────────────────────────────────────────────────

    def _wait_for_joystick(self, pygame):
        """Block until a joystick is detected or the thread is asked to stop."""
        first = True
        while not self._stop.is_set():
            try:
                pygame.joystick.quit()
                pygame.joystick.init()
            except Exception as exc:
                log("local_gp", f"joystick subsystem error: {exc}")
                self._stop.wait(timeout=JOYSTICK_RETRY_SEC)
                continue

            if pygame.joystick.get_count() > 0:
                try:
                    js = pygame.joystick.Joystick(0)
                    js.init()
                    log("local_gp",
                        f"joystick: {js.get_name()} "
                        f"axes={js.get_numaxes()} btns={js.get_numbuttons()} "
                        f"hats={js.get_numhats()}")
                    return js
                except pygame.error as exc:
                    log("local_gp", f"joystick init failed: {exc}")

            if first:
                log("local_gp", "no joystick — waiting…")
                first = False
            self._stop.wait(timeout=JOYSTICK_RETRY_SEC)
        return None

    @staticmethod
    def _select_mapping(js) -> dict:
        """Pick a profile by joystick name and OS — identical to operator."""
        name = (js.get_name() or "").strip().lower()
        n_btn = js.get_numbuttons()
        current_os = platform.system()

        if "ultimate 2 wireless" in name and current_os == "Linux":
            key = "8bitdo_ultimate2_wireless"
        elif "ultimate wireless controller for pc" in name and current_os == "Linux":
            key = "8bitdo_ultimate_wireless_pc"
        elif "ultimate 2 wireless" in name and current_os == "Windows":
            key = "8bitdo_ultimate2_wireless_windows"
        elif n_btn <= 12:
            # Ultimate 2 reports ~11 buttons via the legacy joystick API
            key = "8bitdo_ultimate2_wireless"
        else:
            key = DEFAULT_MAPPING_KEY

        mapping = GAMEPAD_MAPPINGS[key]
        log("local_gp", f"mapping = {key}  (joystick: {js.get_name()!r})")
        log("local_gp",
            f"  steer={mapping['axis_steer']} signal={mapping['axis_signal']} "
            f"sound={mapping['axis_sound']} "
            f"head_lr={mapping['axis_head_lr']} head_ud={mapping['axis_head_ud']}  "
            f"A/B/X/Y={mapping['btn_a']}/{mapping['btn_b']}/"
            f"{mapping['btn_x']}/{mapping['btn_y']}  "
            f"lights_on/off={mapping['btn_lights_on']}/{mapping['btn_lights_off']}")
        return mapping

    @staticmethod
    def _read_btn(js, idx: int) -> int:
        if idx is None or idx < 0:
            return 0
        return js.get_button(idx) if js.get_numbuttons() > idx else 0

    @staticmethod
    def _read_axis(pygame, js, idx: int) -> float:
        if js.get_numaxes() <= idx:
            return 0.0
        try:
            v = js.get_axis(idx)
        except pygame.error:
            return 0.0
        # Some drivers report raw int counts instead of normalized [-1, 1]
        if abs(v) > 1.5:
            v = v / 32767.0
        return _clamp(v, -1.0, 1.0)

    def _read_axis_counts(self, pygame, js, idx: int) -> int:
        return int(round(self._read_axis(pygame, js, idx) * 32767.0))

    # ── dispatcher shortcuts ─────────────────────────────────────────────────

    def _emit_event(self, pkt: dict) -> None:
        """Send a port-57000-style event packet through the orchestrator."""
        pkt["_local"] = True
        try:
            self._on_events(pkt, ("local", 0), -1)
        except Exception as exc:
            log("local_gp", f"events dispatch error: {exc}")

    def _emit_tts(self, text: str, seq: int) -> None:
        """Send a port-57001-style TTS packet through the orchestrator."""
        pkt = {
            "type":   "stt",
            "seq":    seq,
            "ts":     time.time(),
            "text":   text,
            "_local": True,
        }
        try:
            self._on_tts(pkt, ("local", 0), -1)
        except Exception as exc:
            log("local_gp", f"tts dispatch error: {exc}")