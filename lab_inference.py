#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lab_inference_udp.py — Deploy the trained ACT policy on the REVO Scout AGV
via UDP (same port the gamepad uses). No rclpy, no MotionController.

This is for the OLD model trained on UNSCALED (raw) ang_z data.

How it differs from lab_inference.py
------------------------------------
  · Does NOT use rclpy or MotionController.
  · Sends commands as JSON over UDP to 127.0.0.1:55999 — the exact port and
    packet format the gamepad sender uses. teleop.py's UDP motion listener
    receives them and calls motion.command(lin, ang, locked, brake) on its
    side, which applies ang_z_scale (×0.20) internally.
  · Therefore we send RAW ang_z — identical to what the gamepad would send.
    The policy was trained on raw ang_z, predicts raw ang_z → send directly.
  · Observation feedback (obs ang_z fed back into the policy) is the last RAW
    ang_z we sent, tracked locally (no motion.state() available without rclpy).

  IMPORTANT: teleop.py MUST be running to receive these packets and publish
  to /cmd_vel. This script only sends UDP; it does not touch ROS2 itself.

Packet format (matches teleop.py dispatcher, lines 572-576):
    {"lin_x": <float>, "ang_z": <float>, "robot_lock": <bool>}

Usage
-----
    # Dry-run — print predictions, send nothing:
    python3 lab_inference_udp.py \
        --policy-path /path/to/checkpoints/080000/pretrained_model \
        --dataset-repo-id revolabs/scout_dataset_03 \
        --device cpu --duration 60

    # Live — send to robot over UDP (teleop.py must be running):
    python3 lab_inference_udp.py \
        --policy-path /path/to/checkpoints/080000/pretrained_model \
        --dataset-repo-id revolabs/scout_dataset_03 \
        --device cpu --duration 60 \
        --send \
        --temporal-ensemble-coeff 0.01 \
        --ang-deadband 0.15
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

# ── repo root on sys.path ──────────────────────────────────────────────────
_REPO = Path(__file__).resolve().parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

# ── LAB ───────────────────────────────────────────────────────────────────
from LAB.common  import log
from LAB.config  import LabConfig
from LAB.sensors import GpsReader

# ── LeRobot ────────────────────────────────────────────────────────────────
from lerobot.configs.policies           import PreTrainedConfig
from lerobot.datasets.lerobot_dataset   import LeRobotDatasetMetadata
from lerobot.datasets.utils             import build_dataset_frame
from lerobot.policies.factory           import make_policy, make_pre_post_processors
from lerobot.policies.utils             import make_robot_action
from lerobot.processor                  import make_default_processors
from lerobot.processor.rename_processor import rename_stats
from lerobot.utils.constants            import ACTION, OBS_STR
from lerobot.utils.control_utils        import predict_action
from lerobot.utils.utils                import get_safe_torch_device, init_logging


# ── Constants ──────────────────────────────────────────────────────────────
CAMERA_KEY             = "front"
BLANK_FRAME_THRESHOLD  = 5.0
BLANK_FRAME_MAX_CONSEC = 30


# ══════════════════════════════════════════════════════════════════════════════
#  SCALE NOTE (OLD MODEL — UNSCALED DATA)
#  ─────────────────────────────────────────────────────────────────────────
#  The dataset this model was trained on stored RAW ang_z (the gamepad's
#  pre-scale value, ±3.5 rad/s range). So:
#    · Policy observation state ang_z  = RAW  → feed raw back as obs.
#    · Policy output ang_z             = RAW  → send raw over UDP.
#
#  teleop.py receives the UDP packet and calls motion.command(lin, ang, ...),
#  which multiplies ang by ang_z_scale (0.20) before /cmd_vel. So we send the
#  same raw value the gamepad would send — no scaling on our side.
#
#  Observation feedback without rclpy:
#    There is no motion.state() to read. We track the last RAW ang_z we sent
#    in self._last_sent_ang and feed THAT back as the next obs — exactly what
#    motion.state() would have returned (it echoes the last commanded value).
# ══════════════════════════════════════════════════════════════════════════════


# ── UDP command sender ──────────────────────────────────────────────────────

class UdpMotionSender:
    """Sends JSON motion packets to teleop.py's UDP motion port (55999).

    Packet format matches the gamepad sender / teleop dispatcher:
        {"lin_x": float, "ang_z": float, "robot_lock": bool}
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 55999) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log("inference", f"UDP sender → {host}:{port}")

    def send(self, lin_x: float, ang_z: float, robot_lock: bool = False) -> int:
        """Send one motion packet. Returns bytes sent (0 on failure)."""
        payload = json.dumps({
            "lin_x":      float(lin_x),
            "ang_z":      float(ang_z),
            "robot_lock": bool(robot_lock),
            "origin":     "ai", 
        }).encode("utf-8")
        try:
            return self._sock.sendto(payload, self._addr)
        except OSError as exc:
            log("inference", f"UDP send failed: {exc}")
            return 0

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:
            pass


# ── Policy pipeline ────────────────────────────────────────────────────────

def build_policy_pipeline(
    policy_path:     str,
    dataset_repo_id: str,
    device:          str = "cuda",
    rename_map:      Optional[dict] = None,
):
    if rename_map is None:
        rename_map = {}

    ds_meta = LeRobotDatasetMetadata(dataset_repo_id)
    _, robot_action_processor, robot_observation_processor = make_default_processors()

    policy_cfg = PreTrainedConfig.from_pretrained(policy_path)
    policy_cfg.device          = device
    policy_cfg.pretrained_path = policy_path

    policy = make_policy(policy_cfg, ds_meta=ds_meta)

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg      = policy_cfg,
        pretrained_path = policy_path,
        dataset_stats   = rename_stats(ds_meta.stats, rename_map),
        preprocessor_overrides = {
            "device_processor":              {"device": device},
            "rename_observations_processor": {"rename_map": rename_map},
        },
    )

    image_key = f"{OBS_STR}.images.{CAMERA_KEY}"
    if image_key not in ds_meta.features:
        available = [k for k in ds_meta.features if "image" in k.lower()]
        raise RuntimeError(
            f"Camera key '{image_key}' not found in dataset features. "
            f"Available image keys: {available}. Update CAMERA_KEY."
        )

    return (
        policy, preprocessor, postprocessor,
        robot_action_processor, robot_observation_processor,
        ds_meta.features,
    )


# ── Frame validator ────────────────────────────────────────────────────────

class FrameValidator:
    def __init__(self, expected_h, expected_w,
                 blank_thresh=BLANK_FRAME_THRESHOLD, max_consec=BLANK_FRAME_MAX_CONSEC):
        self.expected_shape = (expected_h, expected_w, 3)
        self.blank_thresh   = blank_thresh
        self.max_consec     = max_consec
        self.n_total = self.n_none = self.n_wrong_shape = self.n_blank = self.n_ok = 0
        self._consec_bad = 0

    def validate(self, frame):
        self.n_total += 1
        if frame is None:
            self.n_none += 1; self._consec_bad += 1; self._check_halt()
            return False, "frame is None"
        if frame.shape != self.expected_shape:
            self.n_wrong_shape += 1; self._consec_bad += 1; self._check_halt()
            return False, f"wrong shape {frame.shape}, expected {self.expected_shape}"
        mean_px = float(frame.mean())
        if mean_px < self.blank_thresh:
            self.n_blank += 1; self._consec_bad += 1; self._check_halt()
            return False, f"blank frame mean_px={mean_px:.1f} < {self.blank_thresh}"
        self.n_ok += 1; self._consec_bad = 0
        return True, ""

    def summary(self):
        return (f"total={self.n_total} ok={self.n_ok} none={self.n_none} "
                f"wrong_shape={self.n_wrong_shape} blank={self.n_blank}")

    def _check_halt(self):
        if self._consec_bad >= self.max_consec:
            raise RuntimeError(
                f"{self._consec_bad} consecutive bad frames — halting. {self.summary()}"
            )


# ── Raw observation builder ────────────────────────────────────────────────

def build_raw_observation(frame_bgr, lin_x, ang_z, gps_data):
    """
    Identical structure to data_convert_agv.py.
    lin_x and ang_z are RAW (unscaled) — matches the old dataset's obs state.
    """
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    lat = float(gps_data.get("gps_latitude",  0.0) or 0.0)
    lon = float(gps_data.get("gps_longitude", 0.0) or 0.0)
    ori = float(gps_data.get("orientation",   0.0) or 0.0)
    return {
        "lin_x":       lin_x,
        "ang_z":       ang_z,      # RAW — matches old (unscaled) dataset
        "lat":         lat,
        "long":        lon,
        "orientation": ori,
        CAMERA_KEY:    frame_rgb,
    }


# ── Main inference class ───────────────────────────────────────────────────

class LabInferenceUDP:

    def __init__(
        self,
        policy_path:             str,
        dataset_repo_id:         str,
        device:                  str,
        cfg:                     LabConfig,
        send:                    bool  = False,
        ang_deadband:            float = 0.0,
        temporal_ensemble_coeff: Optional[float] = None,
        duration_s:              Optional[float] = None,
        udp_host:                str   = "127.0.0.1",
        udp_port:                int   = 55999,
    ) -> None:
        self._cfg          = cfg
        self._send         = send
        self._ang_deadband = ang_deadband
        self._stop         = threading.Event()
        self._duration_s   = duration_s

        # Observation feedback: last RAW ang_z and lin_x we sent (no motion.state()).
        # This mirrors what motion.state() would echo back: the last commanded value.
        self._last_sent_lin = 0.0
        self._last_sent_ang = 0.0

        # ── 1. Policy pipeline ────────────────────────────────────────────
        log("inference", f"loading policy: {policy_path}")
        (
            self._policy,
            self._preprocessor,
            self._postprocessor,
            _robot_action_processor,
            self._robot_obs_processor,
            self._features,
        ) = build_policy_pipeline(
            policy_path     = policy_path,
            dataset_repo_id = dataset_repo_id,
            device          = device,
        )

        # ── 2. Temporal ensembling ────────────────────────────────────────
        if temporal_ensemble_coeff is not None:
            self._enable_temporal_ensembling(temporal_ensemble_coeff)

        # ── 3. Frame validator ────────────────────────────────────────────
        self._validator = FrameValidator(
            expected_h = cfg.record_height,
            expected_w = cfg.record_width,
        )

        # ── 4. UDP sender (replaces MotionController) ──────────────────────
        self._sender = UdpMotionSender(host=udp_host, port=udp_port)
        if send:
            # Start locked for safety
            self._sender.send(0.0, 0.0, robot_lock=True)

        # ── 5. GPS ────────────────────────────────────────────────────────
        self._gps = GpsReader(udp_host=cfg.gps_udp_host, udp_port=cfg.gps_udp_port)
        self._gps.start()

        # ── 6. Camera ─────────────────────────────────────────────────────
        self._frame_bus_reader = None
        self._cameras          = None

        cam_cfg = next((c for c in cfg.cameras if c.name == cfg.record_camera_name), None)
        if cam_cfg is None:
            raise RuntimeError(f"Camera {cfg.record_camera_name!r} not in cfg.cameras")

        if cam_cfg.publish_frames:
            self._frame_bus_reader = self._try_attach_frame_bus(cam_cfg.name)

        if self._frame_bus_reader is None:
            log("inference", f"camera {cam_cfg.name!r}: direct V4L2 ({cam_cfg.source})")
            from LAB.cameras import MultiCameraCapture
            self._cameras = MultiCameraCapture.from_configs([cam_cfg])
            if not self._cameras.has(cam_cfg.name):
                raise RuntimeError(
                    f"Cannot open camera {cam_cfg.name!r} at {cam_cfg.source}. "
                    f"Is teleop.py already holding the V4L2 device?"
                )

        self._camera_name = cam_cfg.name
        log("inference", "init complete")

    # ── Public ─────────────────────────────────────────────────────────────

    def run(self) -> None:
        mode = "SENDING UDP TO ROBOT" if self._send else "DRY RUN (print only)"
        log("inference", f"starting — {mode}  deadband={self._ang_deadband}")

        if self._send:
            # Unlock
            self._sender.send(0.0, 0.0, robot_lock=False)

        self._policy.reset()
        self._preprocessor.reset()
        self._postprocessor.reset()

        interval     = 1.0 / self._cfg.stream_fps
        next_tick    = time.monotonic()
        frame_i      = 0
        t_start      = time.time()
        t_start_mono = time.monotonic()

        print()
        print(f"{'frame':>6}  {'timestamp':>12}  {'frame_mean':>10}  "
              f"{'obs_lin':>8}  {'obs_ang':>10}  "
              f"{'lin_x':>8}  {'ang_z_pred':>12}  {'ang_z_sent':>12}  "
              f"{'→robot':>8}")
        print("─" * 105)

        try:
            while not self._stop.is_set():
                if self._duration_s is not None and (time.monotonic() - t_start_mono) >= self._duration_s:
                    log("inference", f"duration {self._duration_s}s reached — stopping")
                    break

                sleep_for = next_tick - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                next_tick += interval

                wall_ts = time.time()

                # ── A. Grab and validate frame ─────────────────────────────
                frame_bgr = self._read_frame()
                ok, reason = self._validator.validate(frame_bgr)
                if not ok:
                    log("inference", f"SKIP f={frame_i}: {reason}")
                    frame_i += 1
                    continue

                mean_px = float(frame_bgr.mean())

                # ── B. Observation state — RAW (unscaled), last sent value ──
                # No motion.state() without rclpy. We echo the last RAW ang_z
                # we sent, which is exactly what motion.state() would return.
                if self._send:
                    lin_x_obs = self._last_sent_lin
                    ang_z_obs = self._last_sent_ang   # RAW — matches old dataset
                else:
                    lin_x_obs, ang_z_obs = 0.0, 0.0

                gps_data = self._gps.get()

                # ── C. Build observation ───────────────────────────────────
                raw_obs = build_raw_observation(
                    frame_bgr = frame_bgr,
                    lin_x     = lin_x_obs,
                    ang_z     = ang_z_obs,   # RAW
                    gps_data  = gps_data,
                )
                obs_processed     = self._robot_obs_processor(raw_obs)
                observation_frame = build_dataset_frame(
                    self._features, obs_processed, prefix=OBS_STR
                )

                # ── D. Policy inference ────────────────────────────────────
                action_values = predict_action(
                    observation   = observation_frame,
                    policy        = self._policy,
                    device        = get_safe_torch_device(self._policy.config.device),
                    preprocessor  = self._preprocessor,
                    postprocessor = self._postprocessor,
                    use_amp       = self._policy.config.use_amp,
                    task          = None,
                    robot_type    = "revobots_agv_follower",
                )
                act_pred = make_robot_action(action_values, self._features)

                # OLD model: policy output is RAW ang_z (unscaled).
                pred_lin = float(act_pred.get("lin_x", 0.0))
                pred_ang = float(act_pred.get("ang_z", 0.0))   # RAW

                # ── E. Deadband ────────────────────────────────────────────
                pred_ang_after_db = pred_ang
                if self._ang_deadband > 0.0 and abs(pred_ang) < self._ang_deadband:
                    pred_ang_after_db = 0.0

                # ── F. Value to send over UDP ──────────────────────────────
                # Send RAW ang_z directly — teleop.py applies *0.20 on its side.
                ang_z_sent = pred_ang_after_db

                # ── G. Print ───────────────────────────────────────────────
                sent_marker = "SEND" if self._send else "----"
                db_marker   = " DB" if pred_ang_after_db != pred_ang else "   "
                print(
                    f"{frame_i:>6d}  "
                    f"{wall_ts:>12.3f}  "
                    f"{mean_px:>10.1f}  "
                    f"{lin_x_obs:>+8.4f}  "
                    f"{ang_z_obs:>+10.5f}  "
                    f"{pred_lin:>+8.4f}  "
                    f"{pred_ang:>+12.5f}{db_marker}  "
                    f"{ang_z_sent:>+12.5f}  "
                    f"{sent_marker:>8}"
                )

                # ── H. Send over UDP ───────────────────────────────────────
                if self._send:
                    self._sender.send(
                        lin_x      = pred_lin,
                        ang_z      = ang_z_sent,   # RAW — teleop applies *0.20
                        robot_lock = False,
                    )
                    # Track for next observation feedback
                    self._last_sent_lin = pred_lin
                    self._last_sent_ang = ang_z_sent

                frame_i += 1

        except RuntimeError as exc:
            print()
            log("inference", f"HALT — {exc}")
        except Exception as exc:
            print()
            log("inference", f"loop error: {exc}")
            import traceback
            traceback.print_exc()
        finally:
            print("─" * 105)
            log("inference", f"frame stats: {self._validator.summary()}")
            self._safe_stop()

    def stop(self) -> None:
        self._stop.set()

    # ── Internal ───────────────────────────────────────────────────────────

    def _enable_temporal_ensembling(self, coeff: float) -> None:
        cfg_p = self._policy.config
        log("inference",
            f"temporal ensembling: coeff={coeff} n_action_steps=1 chunk={cfg_p.chunk_size}")
        cfg_p.temporal_ensemble_coeff = coeff
        cfg_p.n_action_steps = 1
        try:
            from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
            self._policy.temporal_ensembler = ACTTemporalEnsembler(coeff, cfg_p.chunk_size)
        except ImportError:
            log("inference", "WARNING: cannot import ACTTemporalEnsembler")
        self._policy.reset()

    def _try_attach_frame_bus(self, camera_name: str) -> Optional[object]:
        try:
            from LAB.utils.frame_bus import FrameBusReader
            log("inference", f"trying frame bus for {camera_name!r}...")
            rdr = FrameBusReader(camera_name)
            deadline = time.monotonic() + 2.0
            frame = None
            while time.monotonic() < deadline:
                _, frame = rdr.read_latest()
                if frame is not None:
                    break
                time.sleep(0.1)
            if frame is None:
                log("inference",
                    f"frame bus: no frame after 2s — is teleop.py running? "
                    f"Falling back to direct V4L2.")
                rdr.close()
                return None
            tmp = FrameValidator(
                expected_h=self._cfg.record_height,
                expected_w=self._cfg.record_width,
                blank_thresh=BLANK_FRAME_THRESHOLD, max_consec=1,
            )
            ok, reason = tmp.validate(frame)
            if not ok:
                log("inference", f"frame bus first frame invalid ({reason}) — direct V4L2.")
                rdr.close()
                return None
            log("inference", f"frame bus OK: shape={frame.shape}  mean_px={frame.mean():.0f}")
            return rdr
        except Exception as exc:
            log("inference", f"frame bus attach failed ({exc}) — using direct V4L2")
            return None

    def _read_frame(self) -> Optional[np.ndarray]:
        if self._frame_bus_reader is not None:
            _, frame = self._frame_bus_reader.read_latest()
            return frame
        if self._cameras is not None:
            _, frame = self._cameras.read(self._camera_name)
            return frame
        return None

    def _safe_stop(self) -> None:
        log("inference", "stopping — sending zero + lock over UDP")
        if self._send:
            for _ in range(5):
                self._sender.send(0.0, 0.0, robot_lock=True)
                time.sleep(0.05)
        self._sender.close()

        if self._cameras is not None:
            try: self._cameras.stop_all()
            except Exception: pass
        if self._frame_bus_reader is not None:
            try: self._frame_bus_reader.close()
            except Exception: pass
        try: self._gps.stop()
        except Exception: pass

        log("inference", "shutdown complete")


# ── CLI ────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--policy-path",     required=True)
    ap.add_argument("--dataset-repo-id", required=True)
    ap.add_argument("--device",          default="cuda")
    ap.add_argument("--send",            action="store_true",
                    help="Send UDP commands to teleop.py. Without it: print only.")
    ap.add_argument("--ang-deadband",    type=float, default=0.0,
                    help="Zero ang_z below this magnitude (raw units). e.g. 0.15.")
    ap.add_argument("--duration",        type=float, default=None,
                    help="Stop after N seconds. Default: until Ctrl+C.")
    ap.add_argument("--temporal-ensemble-coeff", type=float, default=None,
                    help="Enable temporal ensembling (e.g. 0.01).")
    ap.add_argument("--udp-host",        default="127.0.0.1",
                    help="teleop.py motion listener host. Default 127.0.0.1.")
    ap.add_argument("--udp-port",        type=int, default=None,
                    help="teleop.py motion listener port. Default: cfg.udp_motion_port (55999).")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    init_logging()

    cfg = LabConfig.load_secrets()
    udp_port = args.udp_port if args.udp_port is not None else cfg.udp_motion_port

    print()
    print("=" * 60)
    print("  LAB INFERENCE (UDP — old unscaled model)")
    print("=" * 60)
    print(f"  policy          : {args.policy_path}")
    print(f"  dataset         : {args.dataset_repo_id}")
    print(f"  device          : {args.device}")
    print(f"  --send          : {args.send}  -> {'ROBOT WILL MOVE' if args.send else 'dry run'}")
    print(f"  ang_deadband    : {args.ang_deadband}")
    print(f"  temporal_coeff  : {args.temporal_ensemble_coeff}")
    print(f"  duration        : {args.duration if args.duration else 'unlimited (Ctrl+C)'}")
    print(f"  UDP target      : {args.udp_host}:{udp_port}")
    print(f"  camera          : {cfg.record_camera_name}")
    print(f"  frame shape     : ({cfg.record_height}, {cfg.record_width}, 3)")
    print(f"  GPS UDP         : {cfg.gps_udp_host}:{cfg.gps_udp_port}")
    print()
    print("  NOTE: teleop.py MUST be running to receive UDP packets.")
    print("  ang_z_pred = policy output (RAW, unscaled). Sent directly over UDP.")
    print("  teleop.py applies x0.20 on its side before /cmd_vel.")
    print("  obs_lin/obs_ang = last sent values, fed back as policy obs state.")
    print("=" * 60)
    print()

    inf = LabInferenceUDP(
        policy_path             = args.policy_path,
        dataset_repo_id         = args.dataset_repo_id,
        device                  = args.device,
        cfg                     = cfg,
        send                    = args.send,
        ang_deadband            = args.ang_deadband,
        temporal_ensemble_coeff = args.temporal_ensemble_coeff,
        duration_s              = args.duration,
        udp_host                = args.udp_host,
        udp_port                = udp_port,
    )

    def _on_signal(sig, frame):
        print("\n[inference] interrupt — stopping")
        inf.stop()

    signal.signal(signal.SIGINT,  _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    inf.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())