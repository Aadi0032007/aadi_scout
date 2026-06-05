# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 20:04:03 2026

@author: Aadi
"""
from __future__ import annotations

"""
REVO Scout LAB — unified controller and recorder.

Final behavior:

    Stream:
        starts once at boot
        stays alive during lock/unlock
        stops only on shutdown

    Recording:
        stable unlock -> start fresh recording session
        stable lock   -> stop/finalize recording session

    Lock parsing:
        robot_lock / lock is treated as a STATE, not an event.
        Missing lock field keeps the previous known state.
        Stable debounce prevents flicker from repeatedly starting/stopping recording.

This restores the old seamless streaming behavior while still giving a new
recording session for every unlock period.
"""

import argparse
import json
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

from LAB.audio         import AudioController
from LAB.cameras       import MultiCameraCapture
from LAB.common        import first_float, log, now_mono, truthy
from LAB.config        import LabConfig
from LAB.lights        import LightsController
from LAB.local_gamepad import LocalGamepad
from LAB.motion        import MotionController
from LAB.ptz           import PtzController
from LAB.record        import SessionRecorder
from LAB.sensors       import GpsReader, ImuReader
from LAB.stream        import DailyStream


# ── UDP listener ──────────────────────────────────────────────────────────────

class UdpListener(threading.Thread):
    """One UDP port → one callback."""

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


# ── Source arbitration ────────────────────────────────────────────────────────

class SourceArbiter:
    """
    Tracks active command source.

    Lower priority number wins.
    Example:
        local  = 100
        remote = 200
    """

    def __init__(self, priorities: dict, timeout_sec: float) -> None:
        self._priorities = dict(priorities)
        self._timeout = timeout_sec
        self._last_seen: dict = {k: 0.0 for k in priorities}
        self._lock = threading.Lock()
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

        live.sort()
        self._active = live[0][1]


# ── Lock parsing ──────────────────────────────────────────────────────────────

def parse_lock_state(pkt: dict, last_known_locked: bool) -> tuple[bool, bool]:
    """
    Return:
        locked, lock_field_present

    This avoids the bad pattern:

        truthy(pkt.get("robot_lock") or pkt.get("lock"))

    because that can misread False/missing values.

    If robot_lock / lock is missing, we keep the previous lock state.
    """
    if "robot_lock" in pkt:
        return truthy(pkt["robot_lock"]), True

    if "lock" in pkt:
        return truthy(pkt["lock"]), True

    return last_known_locked, False


# ── Recording manager ─────────────────────────────────────────────────────────

class RecordingManager(threading.Thread):
    """
    Controls recording only.

    Stream is NOT controlled here.
    Stream stays alive continuously for seamless viewing.

    Stable unlock:
        start a fresh recording session

    Stable lock:
        stop/finalize current recording session
    """

    def __init__(
        self,
        recorder: SessionRecorder,
        debounce_sec: float = 0.75,
    ) -> None:
        super().__init__(daemon=True, name="record-manager")
        self._recorder = recorder
        self._debounce_sec = debounce_sec

        self._cv = threading.Condition()
        self._stop_thread = False

        # Robot starts locked.
        self._desired_locked = True
        self._applied_locked = True
        self._last_change = time.monotonic()

    def set_robot_lock(self, locked: bool) -> None:
        locked = bool(locked)

        with self._cv:
            if locked == self._desired_locked:
                return

            self._desired_locked = locked
            self._last_change = time.monotonic()
            self._cv.notify()

    def run(self) -> None:
        while True:
            with self._cv:
                if self._stop_thread:
                    break

                if self._desired_locked == self._applied_locked:
                    self._cv.wait(timeout=0.25)
                    continue

                stable_for = time.monotonic() - self._last_change
                wait_for = self._debounce_sec - stable_for

                if wait_for > 0:
                    self._cv.wait(timeout=wait_for)
                    continue

                target_locked = self._desired_locked

            try:
                if target_locked:
                    self._apply_locked()
                else:
                    self._apply_unlocked()
            except Exception as exc:
                log("teleop", f"record manager error: {exc}")

            with self._cv:
                self._applied_locked = target_locked

    def _apply_unlocked(self) -> None:
        log("teleop", "stable unlock — starting new recording session")

        try:
            if not self._recorder.is_active():
                self._recorder.set_robot_lock(False)
                self._recorder.start()
        except Exception as exc:
            log("teleop", f"recorder start error: {exc}")

    def _apply_locked(self) -> None:
        log("teleop", "stable lock — stopping recording session")

        try:
            self._recorder.set_robot_lock(True)
            self._recorder.stop()
        except Exception as exc:
            log("teleop", f"recorder stop error: {exc}")

    def stop(self) -> None:
        with self._cv:
            self._stop_thread = True
            self._cv.notify()

        try:
            self.join(timeout=2.0)
        except Exception:
            pass

        try:
            self._recorder.set_robot_lock(True)
            self._recorder.stop()
        except Exception as exc:
            log("teleop", f"final recorder stop error: {exc}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="REVO Scout LAB — unified controller")
    ap.add_argument("--env", default=None, help=".env file path, auto-detected if omitted")
    ap.add_argument(
        "--no-local-dongle",
        action="store_true",
        help="Disable the local pygame gamepad",
    )
    args = ap.parse_args()

    cfg = LabConfig.load_secrets(args.env)

    log("teleop", "=" * 60)
    log("teleop", f"cache_dir   = {cfg.cache_dir}")
    log("teleop", f"record_fps  = {cfg.record_fps}")
    log("teleop", f"stream_fps  = {cfg.stream_fps}")
    log(
        "teleop",
        f"ports       = motion:{cfg.udp_motion_port} "
        f"events:{cfg.udp_events_port} tts:{cfg.udp_tts_port}",
    )
    log("teleop", "=" * 60)

    # ── Init ROS2 once ───────────────────────────────────────────────────────

    rclpy_inited = False
    try:
        import rclpy
        rclpy.init(args=None)
        rclpy_inited = True
    except ImportError:
        log("teleop", "rclpy not available — motion will be disabled")

    # ── Start core subsystems ────────────────────────────────────────────────

    cameras = MultiCameraCapture.from_configs(cfg.cameras)

    imu = ImuReader(port=cfg.imu_port_hint, baud=cfg.imu_baud)
    imu.start()

    gps = GpsReader(udp_host=cfg.gps_udp_host, udp_port=cfg.gps_udp_port)
    gps.start()

    motion = MotionController(
        topic=cfg.cmd_vel_topic,
        publish_hz=cfg.motion_publish_hz,
        watchdog_sec=cfg.motion_watchdog_sec,
        ang_z_scale=cfg.ang_z_scale,
    )
    motion.start()

    audio = AudioController(
        piper_model=cfg.piper_model,
        music_dir=cfg.music_dir,
        music_tracks=cfg.music_tracks,
        startup_volume_pct=cfg.startup_volume_pct,
        preferred_sink_patterns=cfg.preferred_sink_patterns,
        preferred_source_patterns=cfg.preferred_source_patterns,
        piper_speaker_id=cfg.piper_speaker_id,
    )
    audio.start()

    lights = LightsController(
        blink_period_sec=cfg.blink_period_sec,
        signal_timeout_sec=cfg.signal_timeout_sec,
        talk_default_duration=cfg.talk_default_duration,
        all_lights_cooldown_sec=cfg.all_lights_cooldown_sec,
        all_lights_blink_sec=cfg.all_lights_blink_sec,
    )
    lights.start()

    ptz = PtzController(
        ip=cfg.ptz_ip,
        port=cfg.ptz_port,
        user=cfg.ptz_user,
        password=cfg.ptz_password or "",
        pan_speed=cfg.ptz_pan_speed,
        tilt_speed=cfg.ptz_tilt_speed,
        loop_hz=cfg.ptz_loop_hz,
        deadband_sec=cfg.ptz_deadband_sec,
        stop_after_sec=cfg.ptz_stop_after_sec,
    )
    ptz.start()
    ptz.set_ptz_unlock_state(True)

    # ── Stream starts once and stays alive ───────────────────────────────────
    #
    # This is the old seamless behavior.
    # Do NOT stop/start stream on lock/unlock.

    stream = DailyStream(
        api_key=cfg.daily_api_key,
        room_url=cfg.daily_room_url,
        room_name=cfg.daily_room_name,
        width=cfg.stream_width,
        height=cfg.stream_height,
        fps=cfg.stream_fps,
        cameras=cameras,
        name_aliases=cfg.camera_name_aliases,
        initial_main_source=cfg.initial_main_source,
        pip_enabled=cfg.pip_enabled,
        pip_left_source=cfg.pip_left_source,
        pip_right_source=cfg.pip_right_source,
        pip_width=cfg.pip_width,
        pip_height=cfg.pip_height,
        pip_margin=cfg.pip_margin,
        pip_gap=cfg.pip_gap,
        pip_stale_sec=cfg.pip_stale_sec,
        pip_show_label=cfg.pip_show_label,
        overlay_speed_badge=cfg.overlay_speed_badge,
        overlay_camera_name=cfg.overlay_camera_name,
        overlay_timestamp=cfg.overlay_timestamp,
        mic_rtsp_url=cfg.mic_rtsp_url,
        mic_rtsp_transport=cfg.mic_rtsp_transport,
        mic_sample_rate=cfg.mic_sample_rate,
        mic_channels=cfg.mic_channels,
        mic_frame_ms=cfg.mic_frame_ms,
        motion_state_fn=motion.state,
    )
    stream.start()
    stream.set_robot_lock(True)

    # ── Recorder starts/stops by debounced lock state ────────────────────────

    recorder = SessionRecorder(
        base_dir=cfg.cache_dir,
        camera_name=cfg.record_camera_name,
        cameras=cameras,
        width=cfg.record_width,
        height=cfg.record_height,
        fps=cfg.record_fps,
        video_bitrate=cfg.record_video_bitrate,
        encoder_preference=cfg.record_encoder_preference,
        motion_state_fn=motion.state,
        imu_get_fn=imu.get,
        gps_get_fn=gps.get,
    )
    recorder.set_robot_lock(True)

    record_manager = RecordingManager(
        recorder=recorder,
        debounce_sec=0.75,
    )
    record_manager.start()

    # ── Source arbitration ───────────────────────────────────────────────────

    arbiter = SourceArbiter(
        priorities={
            "local": cfg.local_dongle_priority,
            "remote": cfg.remote_gamepad_priority,
        },
        timeout_sec=cfg.source_activity_timeout_sec,
    )

    # Edge-detection state.
    prev_state = {
        "a_pressed": False,
        "b_pressed": False,
        "button_8": False,
        "speed_label": None,
    }

    # Last known robot lock state.
    # Robot starts locked.
    lock_state = {
        "locked": True,
    }

    # ── Dispatchers ──────────────────────────────────────────────────────────

    def on_motion_packet(pkt: dict, addr, port: int) -> None:
        """Port 55999 — motion + head + camera + button. Also from local gamepad."""
        source = "local" if pkt.get("_local") else "remote"

        arbiter.report(source)

        if not arbiter.is_active(source):
            return

        # Safe lock parsing.
        locked, lock_present = parse_lock_state(pkt, lock_state["locked"])

        if lock_present and locked != lock_state["locked"]:
            log(
                "teleop",
                f"lock edge from {source} addr={addr}: "
                f"{lock_state['locked']} -> {locked} "
                f"raw_robot_lock={pkt.get('robot_lock', '<missing>')} "
                f"raw_lock={pkt.get('lock', '<missing>')}",
            )

        lock_state["locked"] = locked

        # Motion.
        lin = first_float(pkt, ("lin_x", "linx", "linear_x"))
        ang = first_float(pkt, ("ang_z", "angz", "angular_z"))
        brake = first_float(pkt, ("brake",), default=0.0) > cfg.brake_threshold

        motion.command(lin, ang, locked, brake)

        # Lock state to subsystems.
        lights.set_robot_lock(locked)
        stream.set_robot_lock(locked)

        # Recording session starts/stops only on debounced lock state.
        # Missing lock field does not cause a false edge.
        if lock_present:
            record_manager.set_robot_lock(locked)

        # Camera switch.
        cam = pkt.get("camera") or pkt.get("cam") or pkt.get("video_source")
        if cam:
            try:
                stream.switch_source(str(cam))
            except Exception as exc:
                log("teleop", f"stream camera switch error: {exc}")

        # PTZ direction.
        head = pkt.get("head")
        if head:
            ptz.command(str(head))

        # Speed cycle → capture PTZ home.
        speed_label = pkt.get("speed")
        if speed_label and speed_label != prev_state["speed_label"]:
            if prev_state["speed_label"] is not None:
                ptz.capture_home()
            prev_state["speed_label"] = speed_label

        # Button edges → PTZ home actions.
        a_pressed = truthy(pkt.get("a", False)) or pkt.get("button") == 1
        b_pressed = truthy(pkt.get("b", False)) or pkt.get("button") == 2
        button_8 = pkt.get("button") == cfg.ptz_home_button

        ab_combo = a_pressed and b_pressed
        prev_ab = prev_state["a_pressed"] and prev_state["b_pressed"]

        if ab_combo and not prev_ab:
            ptz.capture_home()

        if button_8 and not prev_state["button_8"]:
            ptz.goto_home()

        prev_state["a_pressed"] = a_pressed
        prev_state["b_pressed"] = b_pressed
        prev_state["button_8"] = button_8

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

    # ── Start UDP listeners ──────────────────────────────────────────────────

    udp_motion = UdpListener(
        cfg.udp_listen_ip,
        cfg.udp_motion_port,
        "motion",
        on_motion_packet,
    )
    udp_events = UdpListener(
        cfg.udp_listen_ip,
        cfg.udp_events_port,
        "events",
        on_events_packet,
    )
    udp_tts = UdpListener(
        cfg.udp_listen_ip,
        cfg.udp_tts_port,
        "tts",
        on_tts_packet,
    )

    udp_motion.start()
    udp_events.start()
    udp_tts.start()

    # ── Local pygame gamepad ─────────────────────────────────────────────────

    local: Optional[LocalGamepad] = None

    if cfg.local_dongle_enabled and not args.no_local_dongle:
        local = LocalGamepad(
            on_motion=on_motion_packet,
            on_events=on_events_packet,
            on_tts=on_tts_packet,
            initial_robot_lock=True,
            priority_value=cfg.local_dongle_priority,
        )
        local.start()

    # ── Signal handling ──────────────────────────────────────────────────────

    running = threading.Event()
    running.set()

    def on_signal(*_) -> None:
        running.clear()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    log(
        "teleop",
        "ready — stream stays alive; stable unlock starts recording; stable lock stops recording",
    )

    try:
        while running.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    # ── Shutdown ─────────────────────────────────────────────────────────────

    log("teleop", "shutting down…")

    # Stop recording manager first so current session finalizes while cameras exist.
    try:
        record_manager.stop()
    except Exception as exc:
        log("teleop", f"record manager stop error: {exc}")

    # Stop stream once, only on shutdown.
    try:
        stream.stop()
    except Exception as exc:
        log("teleop", f"stream stop error: {exc}")

    for sub_name, sub in [
        ("udp_motion", udp_motion),
        ("udp_events", udp_events),
        ("udp_tts", udp_tts),
        ("local", local),
        ("ptz", ptz),
        ("lights", lights),
        ("audio", audio),
        ("motion", motion),
        ("imu", imu),
        ("gps", gps),
        ("cameras", cameras),
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