"""
Session recorder — floor camera as H.264 MP4 + telemetry as JSONL.

One session = one folder under cache_dir:
    session_YYYYMMDD_HHMMSS/
        video.mp4          (H.264, NVENC if available, else V4L2-M2M, else libx264)
        data.jsonl         (one JSON line per video frame, frame-aligned by index)
        session.json       (metadata: start time, FPS, encoder used, etc.)

Recording runs at record_fps. Each tick we:
    1. Pull latest floor camera frame
    2. Pipe BGR bytes into a long-lived ffmpeg subprocess
    3. Write a JSON line with current telemetry tagged by frame_index

Frame index is the single source of truth for alignment — JSONL row N
corresponds to video frame N. No timestamps needed for ML postprocessing,
though we include them anyway for human inspection.

Encoder is auto-probed at startup: tries NVENC → V4L2-M2M → libx264.
First one that opens wins. All produce the same MP4 file format.

robot_lock=True pauses recording (no frames consumed, no JSONL lines written)
so the dataset contains only intentional driving.
"""
from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from .common import log


class SessionRecorder:
    def __init__(
        self,
        base_dir:           str,
        camera_name:        str,
        cameras,                                # MultiCameraCapture
        width:              int,
        height:             int,
        fps:                int,
        video_bitrate:      str,
        encoder_preference: list,
        motion_state_fn:    Optional[Callable[[], tuple]] = None,
        imu_get_fn:         Optional[Callable[[], dict]]  = None,
        gps_get_fn:         Optional[Callable[[], dict]]  = None,
    ) -> None:
        self._base_dir       = Path(base_dir)
        self._camera_name    = camera_name
        self._cameras        = cameras
        self._width          = width
        self._height         = height
        self._fps            = max(1, fps)
        self._video_bitrate  = video_bitrate
        self._encoder_pref   = list(encoder_preference)
        self._motion_state   = motion_state_fn
        self._imu_get        = imu_get_fn
        self._gps_get        = gps_get_fn

        # Per-session state — created lazily on first start()
        self._session_dir:   Optional[Path] = None
        self._video_path:    Optional[Path] = None
        self._jsonl_path:    Optional[Path] = None
        self._ffmpeg:        Optional[subprocess.Popen] = None
        self._encoder_used:  Optional[str]              = None
        self._jsonl_file                                 = None
        self._jsonl_lock     = threading.Lock()

        self._frame_index    = 0
        self._start_unix:    float = 0.0
        self._start_mono:    float = 0.0

        # Recording loop state
        self._active         = False
        self._stop           = threading.Event()
        self._robot_locked   = False
        self._tick_thread:   Optional[threading.Thread] = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Open a new session and begin recording. Idempotent — call again on resume."""
        if self._active:
            return True
        if not self._cameras.has(self._camera_name):
            log("record", f"camera {self._camera_name!r} not available — cannot record")
            return False

        if not self._open_session():
            return False

        self._active = True
        self._stop.clear()
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True, name="rec-tick")
        self._tick_thread.start()
        log("record", f"▶  recording → {self._session_dir}")
        return True

    def stop(self) -> None:
        """Stop recording, flush video + jsonl, close the session."""
        if not self._active:
            return
        self._active = False
        self._stop.set()

        if self._tick_thread is not None:
            try:
                self._tick_thread.join(timeout=2.0)
            except Exception:
                pass
            self._tick_thread = None

        # Close ffmpeg stdin so it finalizes the MP4
        if self._ffmpeg is not None:
            try:
                if self._ffmpeg.stdin is not None:
                    self._ffmpeg.stdin.close()
                self._ffmpeg.wait(timeout=10)
            except Exception:
                try:
                    self._ffmpeg.kill()
                except Exception:
                    pass
            self._ffmpeg = None

        with self._jsonl_lock:
            if self._jsonl_file is not None:
                try:
                    self._jsonl_file.flush()
                    self._jsonl_file.close()
                except Exception:
                    pass
                self._jsonl_file = None

        self._write_session_metadata()
        log("record", f"■  stopped — {self._frame_index} frames → {self._session_dir}")

    def set_robot_lock(self, locked: bool) -> None:
        """When True, pause frame consumption and JSONL writes."""
        self._robot_locked = locked

    def is_active(self) -> bool:
        return self._active

    # ── session lifecycle ─────────────────────────────────────────────────────

    def _open_session(self) -> bool:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_dir = self._base_dir / f"session_{stamp}"
        try:
            self._session_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            log("record", f"cannot create {self._session_dir}: {exc}")
            return False

        self._video_path = self._session_dir / "video.mp4"
        self._jsonl_path = self._session_dir / "data.jsonl"

        # Probe encoders and start ffmpeg
        encoder = self._probe_and_start_ffmpeg(self._video_path)
        if encoder is None:
            log("record", "no H.264 encoder available — cannot record")
            return False
        self._encoder_used = encoder

        # Open JSONL (line-buffered, NOT per-write fsync)
        try:
            self._jsonl_file = open(
                self._jsonl_path, "w", encoding="utf-8", buffering=1,
            )
        except Exception as exc:
            log("record", f"cannot open {self._jsonl_path}: {exc}")
            return False

        self._frame_index = 0
        self._start_unix  = time.time()
        self._start_mono  = time.monotonic()
        return True

    def _probe_and_start_ffmpeg(self, video_path: Path) -> Optional[str]:
        """Try each encoder in order. Return the name of the one that started, or None."""
        for encoder in self._encoder_pref:
            cmd = self._build_ffmpeg_cmd(encoder, video_path)
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                )
                # Give it a moment to fail fast if the encoder is missing
                time.sleep(0.4)
                if proc.poll() is not None:
                    # ffmpeg exited early — encoder not usable
                    err = ""
                    try:
                        err = (proc.stderr.read() or b"").decode("utf-8", errors="ignore")[:200]
                    except Exception:
                        pass
                    log("record", f"encoder {encoder} unavailable: {err.strip() or 'exited early'}")
                    continue
                self._ffmpeg = proc
                log("record", f"encoder = {encoder}")
                return encoder
            except FileNotFoundError:
                log("record", "ffmpeg not found in PATH")
                return None
            except Exception as exc:
                log("record", f"encoder {encoder} start failed: {exc}")
                continue
        return None

    def _build_ffmpeg_cmd(self, encoder: str, video_path: Path) -> list:
        """Build ffmpeg command for raw BGR input → H.264 MP4 output."""
        common_in = [
            "ffmpeg",
            "-hide_banner", "-loglevel", "error",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{self._width}x{self._height}",
            "-r", str(self._fps),
            "-i", "pipe:0",
        ]
        if encoder == "h264_nvenc":
            enc = [
                "-c:v", "h264_nvenc",
                "-preset", "p4",            # balanced quality/speed
                "-b:v", self._video_bitrate,
                "-maxrate", self._video_bitrate,
                "-bufsize", "3000k",
                "-pix_fmt", "yuv420p",
            ]
        elif encoder == "h264_v4l2m2m":
            enc = [
                "-c:v", "h264_v4l2m2m",
                "-b:v", self._video_bitrate,
                "-pix_fmt", "yuv420p",
            ]
        else:   # libx264
            enc = [
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-tune", "zerolatency",
                "-b:v", self._video_bitrate,
                "-pix_fmt", "yuv420p",
            ]
        out = [
            "-movflags", "+faststart",
            "-y",
            str(video_path),
        ]
        return common_in + enc + out

    def _write_session_metadata(self) -> None:
        if self._session_dir is None:
            return
        meta = {
            "session_dir":     str(self._session_dir),
            "start_unix":      self._start_unix,
            "start_iso":       datetime.fromtimestamp(self._start_unix).isoformat(),
            "fps":             self._fps,
            "frame_count":     self._frame_index,
            "duration_sec":    self._frame_index / self._fps if self._fps else 0,
            "encoder":         self._encoder_used,
            "width":           self._width,
            "height":          self._height,
            "video":           "video.mp4",
            "telemetry":       "data.jsonl",
            "camera":          self._camera_name,
        }
        try:
            with open(self._session_dir / "session.json", "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)
        except Exception as exc:
            log("record", f"failed to write session.json: {exc}")

    # ── tick loop ─────────────────────────────────────────────────────────────

    def _tick_loop(self) -> None:
        interval = 1.0 / self._fps
        next_tick = time.monotonic()

        while not self._stop.is_set():
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            next_tick = time.monotonic() + interval

            if self._robot_locked:
                continue

            ts, frame = self._cameras.read(self._camera_name)
            if frame is None:
                continue

            self._write_frame(frame, ts)

    def _write_frame(self, frame: np.ndarray, capture_ts: Optional[float]) -> None:
        # Resize to recording resolution if needed
        if frame.shape[1] != self._width or frame.shape[0] != self._height:
            import cv2
            frame = cv2.resize(frame, (self._width, self._height), interpolation=cv2.INTER_AREA)

        # Push BGR bytes into ffmpeg
        if self._ffmpeg is not None and self._ffmpeg.stdin is not None:
            try:
                self._ffmpeg.stdin.write(frame.tobytes())
            except (BrokenPipeError, OSError) as exc:
                log("record", f"ffmpeg pipe lost: {exc} — stopping recording")
                # Cannot recover this session; mark inactive
                self._active = False
                self._stop.set()
                return

        # Write telemetry row
        row = self._build_row(capture_ts)
        try:
            with self._jsonl_lock:
                if self._jsonl_file is not None:
                    self._jsonl_file.write(json.dumps(row) + "\n")
                    # Note: file is line-buffered (buffering=1), no manual flush per line
        except Exception as exc:
            log("record", f"jsonl write failed: {exc}")

        self._frame_index += 1

    def _build_row(self, capture_ts: Optional[float]) -> dict:
        """One flat dict per frame. Frame index is the alignment key."""
        idx = self._frame_index
        now_unix  = time.time()
        rel_t     = round(now_unix - self._start_unix, 4)

        # Motion state
        lin_x = ang_z = 0.0
        locked = braking = False
        if self._motion_state is not None:
            try:
                lin_x, ang_z, locked, braking = self._motion_state()
            except Exception:
                pass

        # Sensors
        imu_d = self._imu_get() if self._imu_get is not None else {}
        gps_d = self._gps_get() if self._gps_get is not None else {}

        row: dict = {
            "frame_index":      idx,
            "ts_unix":          round(now_unix, 4),
            "ts_capture":       round(capture_ts, 4) if capture_ts else None,
            "relative_time":    rel_t,

            # Motion (the operator action — the label for ML)
            "linear_velocity":  lin_x,
            "angular_velocity": ang_z,
            "robot_locked":     bool(locked),
            "braking":          bool(braking),

            # IMU (flat fields, matches original schema)
            "accelerometer_x":  imu_d.get("accelerometer_x"),
            "accelerometer_y":  imu_d.get("accelerometer_y"),
            "accelerometer_z":  imu_d.get("accelerometer_z"),
            "gyroscope_x":      imu_d.get("gyroscope_x"),
            "gyroscope_y":      imu_d.get("gyroscope_y"),
            "gyroscope_z":      imu_d.get("gyroscope_z"),
            "magnetometer_x":   imu_d.get("magnetometer_x"),
            "magnetometer_y":   imu_d.get("magnetometer_y"),
            "magnetometer_z":   imu_d.get("magnetometer_z"),
            "roll":             imu_d.get("roll"),
            "pitch":            imu_d.get("pitch"),
            "yaw":              imu_d.get("yaw"),

            # GPS
            "gps_latitude":     gps_d.get("gps_latitude"),
            "gps_longitude":    gps_d.get("gps_longitude"),
            "gps_altitude":     gps_d.get("gps_altitude"),
            "gps_fix":          gps_d.get("gps_fix"),
            "gps_satellites":   gps_d.get("gps_satellites"),
            "gps_hdop":         gps_d.get("gps_hdop"),
            "gps_speed_kmh":    gps_d.get("gps_speed_kmh"),
            "orientation":      gps_d.get("orientation"),
            "gps_solution_status": gps_d.get("gps_solution_status"),
            "gps_position_type":   gps_d.get("gps_position_type"),
        }
        return row