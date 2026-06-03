"""
LAB configuration — single source of truth for every tunable value.

Edit the class body to retune the robot. Secrets (Daily API key, PTZ password)
load from LAB/.env so they never end up in git.

Three UDP ports are bound:
    55999 — motion + camera + head + button (the gamepad's primary port)
    57000 — events: lights, signals, audio volume, talk, music
    57001 — TTS text (`type:"stt"`)

The operator code is unchanged — these match what the gamepad sender writes
when invoked with `--robot ELEPHANT`.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


# ── Camera source definition ──────────────────────────────────────────────────

@dataclass
class CameraConfig:
    """One physical camera (RTSP URL or V4L2 device path)."""
    name:   str
    source: str
    width:  int = 640
    height: int = 480
    fps:    int = 15
    rtsp_transport: str = "tcp"   # only used for RTSP sources

    @property
    def is_rtsp(self) -> bool:
        return self.source.startswith("rtsp://")

    def __post_init__(self) -> None:
        if self.rtsp_transport not in ("tcp", "udp"):
            raise ValueError(
                f"{self.name}: rtsp_transport must be 'tcp' or 'udp', "
                f"got {self.rtsp_transport!r}"
            )


# ══════════════════════════════════════════════════════════════════════════════
#  Edit everything below to match your robot
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class LabConfig:

    # ── UDP listener ports ────────────────────────────────────────────────────
    udp_listen_ip:          str = "0.0.0.0"
    udp_motion_port:        int = 55999    # lin_x, ang_z, head, camera, button, robot_lock
    udp_events_port:        int = 57000    # lights, signals, audio, talk, music
    udp_tts_port:           int = 57001    # type:"stt", text

    # ── Source priority arbitration ───────────────────────────────────────────
    local_dongle_priority:  int  = 100     # lower wins
    remote_gamepad_priority: int = 200
    source_activity_timeout_sec: float = 1.0   # silent > this → other source takes over

    # ── Motion ────────────────────────────────────────────────────────────────
    cmd_vel_topic:          str   = "/cmd_vel"
    motion_publish_hz:      int   = 50
    motion_watchdog_sec:    float = 0.30   # stop robot if no command for this long
    ang_z_scale:            float = 0.20   # turning attenuation (matches original)
    brake_threshold:        float = 0.20

    # ── PTZ ───────────────────────────────────────────────────────────────────
    ptz_ip:                 str   = "192.168.10.50"
    ptz_port:               int   = 8000
    ptz_user:               str   = "revolabs"
    ptz_pan_speed:          float = 0.65
    ptz_tilt_speed:         float = 0.55
    ptz_loop_hz:            float = 25.0
    ptz_deadband_sec:       float = 0.05    # don't re-issue same command within this window
    ptz_stop_after_sec:     float = 0.15    # halt if no command for this long
    ptz_home_button:        int   = 8       # gamepad button number that returns to home

    # ── Lights / signals ──────────────────────────────────────────────────────
    blink_period_sec:       float = 0.40    # turn-signal blink period (matches original)
    signal_timeout_sec:     float = 20.0    # turn-signal auto-cancel
    talk_default_duration:  float = 7.0
    all_lights_cooldown_sec: float = 5.0    # absorbs the gamepad's 10× repeat
    all_lights_blink_sec:   float = 5.0     # blink-all-on choreography duration

    # ── Audio ─────────────────────────────────────────────────────────────────
    piper_model:            str   = str(Path.home() / "Revobots" / "piper" / "voices" / "en_GB-northern_english_male-medium.onnx")
    piper_speaker_id:       Optional[int] = None
    music_dir:              str   = str(Path.home() / "Revobots" / "Audio")
    music_tracks: dict      = field(default_factory=lambda: {
        1: "REVOBOTS_Anthem_v1.wav",
        2: "REVO_Track_old1.wav",
        3: "REVO_Track_old2.wav",
    })
    startup_volume_pct:     int   = 100
    preferred_sink_patterns: list = field(default_factory=lambda: [
        "ugreen", "u_green", "usb_audio", "usb-audio", "emeet", "alsa_output.usb-",
    ])
    preferred_source_patterns: list = field(default_factory=lambda: [
        "ugreen", "u_green", "usb_audio", "usb-audio", "emeet", "alsa_input.usb-",
    ])

    # ── Cameras ───────────────────────────────────────────────────────────────
    cameras: list = field(default_factory=lambda: [
        CameraConfig(
            name="orbital",
            source="rtsp://revolabs:revolabs123%40@192.168.10.50:554/h264Preview_01_main",
            width=640, height=480, fps=15, rtsp_transport="tcp",
        ),
        CameraConfig(
            name="ai_front",
            source="rtsp://revolabs:revolabs123%40@192.168.10.52:554/revo_rear_ai_cam/realmonitor?channel=1&subtype=1",
            width=640, height=480, fps=10, rtsp_transport="tcp",
        ),
        CameraConfig(
            name="ai_back",
            source="rtsp://revolabs:revolabs123%40@192.168.10.51:554/revo_front_ai_cam/realmonitor?channel=1&subtype=1",
            width=640, height=480, fps=10, rtsp_transport="tcp",
        ),
        CameraConfig(
            name="floor",
            source="/dev/floor_cam",        # udev symlink — set this up in /etc/udev/rules.d
            width=640, height=480, fps=15,
        ),
    ])

    # The gamepad sends these camera names. Map them to our internal names above.
    # Anything not listed here is passed through unchanged.
    camera_name_aliases: dict = field(default_factory=lambda: {
        "pilot":     "orbital",
        "front":     "ai_front",
        "rear":      "ai_back",
        "ai-front":  "ai_front",
        "ai-back":   "ai_back",
        "aifront":   "ai_front",
        "aiback":    "ai_back",
    })

    # ── Daily streaming ───────────────────────────────────────────────────────
    daily_room_url:         str   = "https://revolabs.daily.co/scoutlab-pilot-cam"
    daily_room_name:        str   = "scoutlab-pilot-cam"
    stream_width:           int   = 640
    stream_height:          int   = 480
    stream_fps:             int   = 15
    initial_main_source:    str   = "floor"   # which camera is shown on startup

    # ── PiP thumbnails on the main stream ─────────────────────────────────────
    pip_enabled:            bool  = True
    pip_left_source:        str   = "orbital"     # pilot on left
    pip_right_source:       str   = "ai_back"     # rear on right
    pip_width:              int   = 192
    pip_height:             int   = 144
    pip_margin:             int   = 12
    pip_gap:                int   = 8
    pip_stale_sec:          float = 0.60          # drop thumbnails older than this
    pip_show_label:         bool  = True

    # Speed/camera-name badges
    overlay_speed_badge:    bool  = True
    overlay_camera_name:    bool  = True
    overlay_timestamp:      bool  = False

    # ── Microphone (RTSP audio from orbital → Daily virtual mic) ──────────────
    mic_rtsp_url:           str   = "rtsp://revolabs:revolabs123%40@192.168.10.50:554/h264Preview_01_sub"
    mic_rtsp_transport:     str   = "tcp"
    mic_sample_rate:        int   = 16000
    mic_channels:           int   = 1
    mic_frame_ms:           int   = 5

    # ── Sensors (direct UART, no journalctl, no ROS2) ─────────────────────────
    imu_port_hint:          str   = "/dev/ttyCH341USB3"
    imu_baud:               int   = 9600
    gps_port_hint:          str   = "/dev/ttyCH341USB2"
    gps_baud:               int   = 115200

    # ── Recording ─────────────────────────────────────────────────────────────
    cache_dir:              str   = os.path.expanduser("~/.cache/scout/lab")
    record_camera_name:     str   = "floor"       # which camera goes into the MP4
    record_fps:             int   = 15            # same as stream — frame-aligned
    record_video_bitrate:   str   = "1500k"       # ffmpeg -b:v
    # Encoder preference order — first available wins. Auto-probed at startup.
    record_encoder_preference: list = field(default_factory=lambda: [
        "h264_nvenc",      # Jetson NVIDIA hw encoder
        "h264_v4l2m2m",    # Jetson V4L2 hw encoder
        "libx264",         # software fallback (still fast at 640x480@15)
    ])

    # ── Local dongle (evdev) ──────────────────────────────────────────────────
    local_dongle_enabled:   bool  = True
    # Device-name fragments that identify a real driving controller (not a mouse/kb).
    local_dongle_name_hints: list = field(default_factory=lambda: [
        "8bitdo", "ultimate", "tgz", "cx 2.4g",
    ])

    # ── Secrets (populated by load_secrets() from LAB/.env) ───────────────────
    daily_api_key:          str   = ""
    ptz_password:           str   = ""

    # ── Loader ────────────────────────────────────────────────────────────────

    @classmethod
    def load_secrets(cls, env_file: Optional[str] = None) -> "LabConfig":
        cfg = cls()

        if env_file is None:
            env_file = str(Path(__file__).parent / ".env")

        secrets = _read_env_file(env_file)

        cfg.daily_api_key = secrets.get("DAILY_API_KEY", "")
        cfg.ptz_password  = secrets.get("PTZ_PASSWORD",  "")

        if secrets:
            print(f"[config] secrets loaded from {env_file}")
        else:
            print(f"[config] no .env at {env_file} — secrets empty")

        return cfg


# ── private helper ────────────────────────────────────────────────────────────

def _read_env_file(path: str) -> dict:
    out: dict = {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[7:]
                if "=" not in line:
                    continue
                key, val = line.split("=", 1)
                out[key.strip()] = val.strip().strip("\"'")
    except FileNotFoundError:
        pass
    except Exception as exc:
        print(f"[config] error reading {path}: {exc}")
    return out