# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 20:04:03 2026

@author: Aadi
"""

"""
REVO Scout LAB — unified controller and recorder.

Single Python process. Replaces all separate systemd services from the
original architecture. Operator gamepad code is unchanged on the PC side.

Listens on three UDP ports (matches the unchanged operator wire format):
    55999  motion + head + camera + button + robot_lock
    57000  lights / signals / audio / talk / music events
    57001  TTS text (type:"stt")

Two command sources with priority arbitration:
    1. Local USB dongle plugged into the Jetson (highest priority via evdev)
    2. Remote gamepad over Tailscale (lower priority)

Press 'r' to toggle recording, or send {"record": true|false} over UDP.

Layout:
    main()
        → load config + secrets
        → init rclpy ONCE (only motion uses it)
        → start cameras, sensors, motion, ptz, lights, audio, stream, recorder
        → bind three UDP listeners
        → bind local evdev (optional)
        → main tick: source arbitration, keyboard, shutdown
        → on exit: stop everything in reverse order
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Optional

_REPO = Path(__file__).resolve().parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from LAB.audio    import AudioController
from LAB.cameras  import MultiCameraCapture
from LAB.common   import first_float, log, now_mono, truthy
from LAB.config   import LabConfig
from LAB.lights   import LightsController
from LAB.motion   import MotionController
from LAB.ptz      import PtzController
from LAB.record   import SessionRecorder
from LAB.sensors  import GpsReader, ImuReader
from LAB.stream   import DailyStream


# ── UDP listener ──────────────────────────────────────────────────────────────

class UdpListener(threading.Thread):
    """One UDP port → one callback. Used three times for the three ports."""

    def __init__(self, host: str, port: int, label: str, on_packet) -> None:
        super().__init__(daemon=True, name=f"udp-{label}")
        self._host = host
        self._port = port
        self._label = label
        self._on_packet = on_packet
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None

    def run(self) -> None:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self._host, self._port))
            self._sock.settimeout(0.2)
        except OSError as exc:
            log("teleop", f"UDP bind failed {self._host}:{self._port} ({self._label}): {exc}")
            return

        log("teleop", f"UDP listener {self._label} on {self._host}:{self._port}")

        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                pkt = json.loads(data.decode("utf-8", errors="replace"))
                if not isinstance(pkt, dict):
                    continue
            except json.JSONDecodeError:
                continue

            try:
                self._on_packet(pkt, addr, self._port)
            except Exception as exc:
                log("teleop", f"{self._label} dispatch error: {exc}")

        try:
            if self._sock is not None:
                self._sock.close()
        except Exception:
            pass

    def stop(self) -> None:
        self._stop.set()


# ── Local dongle (evdev) — highest priority command source ────────────────────

class LocalDongleListener(threading.Thread):
    """
    Reads a USB gamepad plugged directly into the Jetson via evdev.
    Translates evdev events into the same dict format the UDP gamepad sends,
    then routes through the same dispatcher as remote packets — but tagged
    with higher priority so it preempts the remote source.
    """

    def __init__(self, name_hints: list, on_packet) -> None:
        super().__init__(daemon=True, name="local-dongle")
        self._hints = [h.lower() for h in name_hints]
        self._on_packet = on_packet
        self._stop = threading.Event()
        self._dev = None

    def run(self) -> None:
        try:
            from evdev import InputDevice, list_devices, ecodes
        except ImportError:
            log("teleop", "evdev not installed — local dongle disabled (pip install evdev)")
            return

        while not self._stop.is_set():
            self._dev = self._find_device(InputDevice, list_devices)
            if self._dev is None:
                self._stop.wait(timeout=2.0)
                continue

            log("teleop", f"local dongle connected: {self._dev.name}")

            # Latest values that we re-emit as packets
            lin_x = 0.0
            ang_z = 0.0
            head  = "center"
            send_due = now_mono()

            try:
                for event in self._dev.read_loop():
                    if self._stop.is_set():
                        break

                    if event.type == ecodes.EV_ABS:
                        if event.code == ecodes.ABS_RZ:
                            lin_x = self._normalize_axis(event.value)
                        elif event.code == ecodes.ABS_Z:
                            ang_z = -self._normalize_axis(event.value)
                        elif event.code == ecodes.ABS_HAT0X:
                            head = {"-1": "left", "1": "right", "0": "center"}[str(event.value)]
                        elif event.code == ecodes.ABS_HAT0Y:
                            head = {"-1": "up", "1": "down", "0": "center"}[str(event.value)]

                    # Emit at ~25 Hz to match a comfortable rate without flooding
                    if now_mono() >= send_due:
                        send_due = now_mono() + 0.04
                        pkt = {
                            "lin_x":      lin_x,
                            "ang_z":      ang_z,
                            "head":       head,
                            "robot_lock": False,    # local dongle is implicitly trusted
                            "_local":     True,     # marker for the dispatcher
                        }
                        try:
                            self._on_packet(pkt, ("local", 0), -1)
                        except Exception as exc:
                            log("teleop", f"local dispatch error: {exc}")

            except OSError:
                log("teleop", "local dongle disconnected")
                self._dev = None

    @staticmethod
    def _normalize_axis(value: int) -> float:
        """Map evdev axis [0..255, center 128] to [-1.0..+1.0] with deadzone."""
        delta = value - 128
        if abs(delta) < 6:
            return 0.0
        return max(-1.0, min(1.0, delta / 127.0))

    def _find_device(self, InputDevice, list_devices):
        """Find the first evdev device whose name matches a configured hint."""
        for path in list_devices():
            try:
                dev = InputDevice(path)
            except Exception:
                continue
            name_lc = (dev.name or "").lower()
            if any(h in name_lc for h in self._hints):
                # Skip keyboards / mice that may share name fragments
                if any(x in name_lc for x in ("keyboard", "mouse", "consumer", "system")):
                    continue
                return dev
        return None

    def stop(self) -> None:
        self._stop.set()


# ── Source arbitration ────────────────────────────────────────────────────────

class SourceArbiter:
    """
    Tracks which command source is currently active. Lower priority number wins.
    If the winning source goes silent for `timeout_sec`, the next-priority active
    source takes over.
    """

    def __init__(self, priorities: dict, timeout_sec: float) -> None:
        self._priorities  = dict(priorities)            # {"local": 100, "remote": 200}
        self._timeout     = timeout_sec
        self._last_seen: dict = {k: 0.0 for k in priorities}
        self._lock        = threading.Lock()
        self._active: Optional[str] = None

    def report(self, source: str) -> None:
        with self._lock:
            self._last_seen[source] = now_mono()
            self._update_active_locked()

    def is_active(self, source: str) -> bool:
        with self._lock:
            self._update_active_locked()
            return self._active == source

    def active(self) -> Optional[str]:
        with self._lock:
            self._update_active_locked()
            return self._active

    def _update_active_locked(self) -> None:
        now = now_mono()
        live = [
            (self._priorities[s], s)
            for s, ts in self._last_seen.items()
            if (now - ts) <= self._timeout
        ]
        if not live:
            self._active = None
            return
        live.sort()                  # lowest priority wins
        self._active = live[0][1]


# ── Keyboard 'r' toggle ───────────────────────────────────────────────────────

def start_keyboard_listener(on_r) -> Optional[object]:
    try:
        from pynput import keyboard
    except ImportError:
        log("teleop", "pynput not installed — 'r' toggle unavailable")
        return None

    def on_press(key):
        if getattr(key, "char", None) == "r":
            on_r()

    listener = keyboard.Listener(on_press=on_press)
    listener.daemon = True
    listener.start()
    return listener


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="REVO Scout LAB — unified controller")
    ap.add_argument("--env", default=None, help=".env file path (auto-detected)")
    ap.add_argument("--no-local-dongle", action="store_true", help="Disable evdev local dongle")
    args = ap.parse_args()

    cfg = LabConfig.load_secrets(args.env)

    log("teleop", "=" * 60)
    log("teleop", f"cache_dir   = {cfg.cache_dir}")
    log("teleop", f"record_fps  = {cfg.record_fps}")
    log("teleop", f"stream_fps  = {cfg.stream_fps}")
    log("teleop", f"ports       = motion:{cfg.udp_motion_port} events:{cfg.udp_events_port} tts:{cfg.udp_tts_port}")
    log("teleop", "=" * 60)

    # ── Init ROS2 once for the whole process ─────────────────────────────────
    rclpy_inited = False
    try:
        import rclpy
        rclpy.init(args=None)
        rclpy_inited = True
    except ImportError:
        log("teleop", "rclpy not available — motion will be disabled")

    # ── Start subsystems in dependency order ─────────────────────────────────

    cameras = MultiCameraCapture.from_configs(cfg.cameras)

    imu = ImuReader(port=cfg.imu_port_hint, baud=cfg.imu_baud)
    imu.start()

    gps = GpsReader(port=cfg.gps_port_hint, baud=cfg.gps_baud)
    gps.start()

    motion = MotionController(
        topic         = cfg.cmd_vel_topic,
        publish_hz    = cfg.motion_publish_hz,
        watchdog_sec  = cfg.motion_watchdog_sec,
        ang_z_scale   = cfg.ang_z_scale,
    )
    motion.start()

    audio = AudioController(
        piper_model               = cfg.piper_model,
        music_dir                 = cfg.music_dir,
        music_tracks              = cfg.music_tracks,
        startup_volume_pct        = cfg.startup_volume_pct,
        preferred_sink_patterns   = cfg.preferred_sink_patterns,
        preferred_source_patterns = cfg.preferred_source_patterns,
        piper_speaker_id          = cfg.piper_speaker_id,
    )
    audio.start()

    lights = LightsController(
        blink_period_sec        = cfg.blink_period_sec,
        signal_timeout_sec      = cfg.signal_timeout_sec,
        talk_default_duration   = cfg.talk_default_duration,
        all_lights_cooldown_sec = cfg.all_lights_cooldown_sec,
        all_lights_blink_sec    = cfg.all_lights_blink_sec,
    )
    lights.start()

    ptz = PtzController(
        ip             = cfg.ptz_ip,
        port           = cfg.ptz_port,
        user           = cfg.ptz_user,
        password       = cfg.ptz_password or "",
        pan_speed      = cfg.ptz_pan_speed,
        tilt_speed     = cfg.ptz_tilt_speed,
        loop_hz        = cfg.ptz_loop_hz,
        deadband_sec   = cfg.ptz_deadband_sec,
        stop_after_sec = cfg.ptz_stop_after_sec,
    )
    ptz.start()
    ptz.set_ptz_unlock_state(True)   # PTZ is independently unlocked; can pan even when robot is locked

    stream = DailyStream(
        api_key             = cfg.daily_api_key,
        room_url            = cfg.daily_room_url,
        room_name           = cfg.daily_room_name,
        width               = cfg.stream_width,
        height              = cfg.stream_height,
        fps                 = cfg.stream_fps,
        cameras             = cameras,
        name_aliases        = cfg.camera_name_aliases,
        initial_main_source = cfg.initial_main_source,
        pip_enabled         = cfg.pip_enabled,
        pip_left_source     = cfg.pip_left_source,
        pip_right_source    = cfg.pip_right_source,
        pip_width           = cfg.pip_width,
        pip_height          = cfg.pip_height,
        pip_margin          = cfg.pip_margin,
        pip_gap             = cfg.pip_gap,
        pip_stale_sec       = cfg.pip_stale_sec,
        pip_show_label      = cfg.pip_show_label,
        overlay_speed_badge = cfg.overlay_speed_badge,
        overlay_camera_name = cfg.overlay_camera_name,
        overlay_timestamp   = cfg.overlay_timestamp,
        mic_rtsp_url        = cfg.mic_rtsp_url,
        mic_rtsp_transport  = cfg.mic_rtsp_transport,
        mic_sample_rate     = cfg.mic_sample_rate,
        mic_channels        = cfg.mic_channels,
        mic_frame_ms        = cfg.mic_frame_ms,
        motion_state_fn     = motion.state,
    )
    stream.start()

    recorder = SessionRecorder(
        base_dir           = cfg.cache_dir,
        camera_name        = cfg.record_camera_name,
        cameras            = cameras,
        width              = cfg.record_width,
        height             = cfg.record_height,
        fps                = cfg.record_fps,
        video_bitrate      = cfg.record_video_bitrate,
        encoder_preference = cfg.record_encoder_preference,
        motion_state_fn    = motion.state,
        imu_get_fn         = imu.get,
        gps_get_fn         = gps.get,
    )

    # ── Recording control ─────────────────────────────────────────────────────

    rec_lock = threading.Lock()

    def toggle_recording() -> None:
        with rec_lock:
            if recorder.is_active():
                recorder.stop()
            else:
                recorder.start()

    def set_recording(active: bool) -> None:
        with rec_lock:
            if active and not recorder.is_active():
                recorder.start()
            elif (not active) and recorder.is_active():
                recorder.stop()

    # ── Source arbitration ───────────────────────────────────────────────────

    arbiter = SourceArbiter(
        priorities = {
            "local":  cfg.local_dongle_priority,
            "remote": cfg.remote_gamepad_priority,
        },
        timeout_sec = cfg.source_activity_timeout_sec,
    )

    # ── Per-source state for edge detection ──────────────────────────────────
    # The orchestrator needs to track button states across packets to detect:
    #   - A+B combo edge → ptz.capture_home()
    #   - button==8 edge → ptz.goto_home()
    #   - speed-label change → ptz.capture_home()
    prev_state = {
        "a_pressed":     False,
        "b_pressed":     False,
        "button_8":      False,
        "speed_label":   None,
        "rec_flag":      None, 
    }

    # ── Dispatchers ──────────────────────────────────────────────────────────

    def on_motion_packet(pkt: dict, addr, port: int) -> None:
        """Port 55999 — motion + head + camera + button. Also from local dongle."""
        source = "local" if pkt.get("_local") else "remote"
        arbiter.report(source)
        if not arbiter.is_active(source):
            return

        # --- motion gates ---
        lin    = first_float(pkt, ("lin_x", "linx", "linear_x"))
        ang    = first_float(pkt, ("ang_z", "angz", "angular_z"))
        locked = truthy(pkt.get("robot_lock") or pkt.get("lock"))
        brake  = first_float(pkt, ("brake",), default=0.0) > cfg.brake_threshold

        motion.command(lin, ang, locked, brake)
        lights.set_robot_lock(locked)
        stream.set_robot_lock(locked)
        recorder.set_robot_lock(locked)

        # --- camera switch ---
        cam = pkt.get("camera") or pkt.get("cam") or pkt.get("video_source")
        if cam:
            stream.switch_source(str(cam))

        # --- head / PTZ direction ---
        head = pkt.get("head")
        if head:
            ptz.command(str(head))

        # --- speed-cycle capture-home detection ---
        speed_label = pkt.get("speed")
        if speed_label and speed_label != prev_state["speed_label"]:
            if prev_state["speed_label"] is not None:
                ptz.capture_home()
            prev_state["speed_label"] = speed_label

        # --- button edges → PTZ home actions ---
        a_pressed = truthy(pkt.get("a", False)) or pkt.get("button") == 1
        b_pressed = truthy(pkt.get("b", False)) or pkt.get("button") == 2
        button_8  = pkt.get("button") == cfg.ptz_home_button

        ab_combo = a_pressed and b_pressed
        prev_ab  = prev_state["a_pressed"] and prev_state["b_pressed"]
        if ab_combo and not prev_ab:
            ptz.capture_home()

        if button_8 and not prev_state["button_8"]:
            ptz.goto_home()

        prev_state["a_pressed"] = a_pressed
        prev_state["b_pressed"] = b_pressed
        prev_state["button_8"]  = button_8

        # --- UDP-initiated recording toggle ---
        # Only act on edges (transitions), not steady-state values.
        # Operator may resend "record": true at 50 Hz; we ignore repeats.
        rec_flag = pkt.get("record")
        if rec_flag is not None:
            rec_flag = bool(rec_flag)
            if rec_flag != prev_state.get("rec_flag"):
                if rec_flag:
                    set_recording(True)
                else:
                    set_recording(False)
                prev_state["rec_flag"] = rec_flag

    def on_events_packet(pkt: dict, addr, port: int) -> None:
        """Port 57000 — lights, signals, audio volume, talk blink, music."""
        event = (pkt.get("event") or "").strip().lower()

        if event in ("lights", "signals", "talk"):
            lights.command(pkt)

        if event == "audio":
            vol = pkt.get("volume_pct")
            if vol is not None:
                audio.set_volume(int(vol))

        if event == "music":
            action = (pkt.get("action") or "").strip().lower()
            if action in ("play", "pla2"):
                track = pkt.get("track")
                if track is not None:
                    audio.play_music(int(track))

    def on_tts_packet(pkt: dict, addr, port: int) -> None:
        """Port 57001 — TTS text."""
        if pkt.get("type") == "stt":
            text = pkt.get("text", "")
            if text:
                audio.speak(str(text))

    # ── Start listeners ──────────────────────────────────────────────────────

    udp_motion = UdpListener(cfg.udp_listen_ip, cfg.udp_motion_port, "motion", on_motion_packet)
    udp_events = UdpListener(cfg.udp_listen_ip, cfg.udp_events_port, "events", on_events_packet)
    udp_tts    = UdpListener(cfg.udp_listen_ip, cfg.udp_tts_port,    "tts",    on_tts_packet)
    udp_motion.start()
    udp_events.start()
    udp_tts.start()

    local: Optional[LocalDongleListener] = None
    if cfg.local_dongle_enabled and not args.no_local_dongle:
        local = LocalDongleListener(cfg.local_dongle_name_hints, on_motion_packet)
        local.start()

    kb = start_keyboard_listener(toggle_recording)

    # ── Signal handling and main wait ─────────────────────────────────────────

    running = threading.Event()
    running.set()

    def on_signal(*_):
        running.clear()

    signal.signal(signal.SIGINT,  on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    log("teleop", "ready — 'r' to toggle recording, Ctrl-C to quit")

    try:
        try:
            while running.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
    finally:
        # ── Shutdown ─────────────────────────────────────────────────────────────
        log("teleop", "shutting down…")

        try:
            if recorder.is_active():
                recorder.stop()
        except Exception as exc:
            log("teleop", f"recorder stop error: {exc}")

    # ── Shutdown ─────────────────────────────────────────────────────────────

    log("teleop", "shutting down…")

    try:
        if recorder.is_active():
            recorder.stop()
    except Exception as exc:
        log("teleop", f"recorder stop error: {exc}")

    for sub_name, sub in [
        ("udp_motion", udp_motion),
        ("udp_events", udp_events),
        ("udp_tts",    udp_tts),
        ("local",      local),
        ("kb",         kb),
        ("stream",     stream),
        ("ptz",        ptz),
        ("lights",     lights),
        ("audio",      audio),
        ("motion",     motion),
        ("imu",        imu),
        ("gps",        gps),
        ("cameras",    cameras),
    ]:
        if sub is None:
            continue
        try:
            if hasattr(sub, "stop"):
                sub.stop()
            elif hasattr(sub, "stop_all"):
                sub.stop_all()
        except Exception as exc:
            log("teleop", f"{sub_name} stop error: {exc}")

    if rclpy_inited:
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass

    log("teleop", "done.")


if __name__ == "__main__":
    main()