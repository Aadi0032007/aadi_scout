# REVO Scout LAB

Unified teleoperation, Daily streaming, lights, PTZ, audio, sensors, RTK GPS, and dataset recording for the REVO Scout / Segway Scout XT robot.

The LAB stack is designed around one foreground Python controller, `LAB/teleop.py`, plus a separate GPS/RTK supervisor when centimeter-grade GNSS is needed. The operator PC gamepad protocol is preserved: motion, camera selection, head/PTZ, lights, audio, music, TTS, and recording commands still arrive over the same UDP fields and ports.

---

## Current architecture

```text
aadi_scout/
├── LAB/
│   ├── __init__.py              # Package marker
│   ├── audio.py                 # PulseAudio setup, Piper TTS, music playback
│   ├── cameras.py               # RTSP + V4L2 capture with latest-frame buffers
│   ├── common.py                # Shared helpers: log, time, value coercion
│   ├── config.py                # LabConfig and CameraConfig
│   ├── lights.py                # 4-channel USB HID relay controller
│   ├── motion.py                # ROS 2 /cmd_vel publisher and watchdog
│   ├── ptz.py                   # ONVIF PTZ control with home capture/return
│   ├── record.py                # MP4 + frame-aligned JSONL session recorder
│   ├── sensors.py               # WIT/JY901 IMU UART + GPS UDP/NMEA reader
│   ├── stream.py                # Daily virtual camera/mic, PiP, overlays
│   ├── teleop.py                # Main orchestrator and UDP dispatch
│   ├── utils/
│   │   └── gps_mux.py           # Single-owner GNSS serial mux for Polaris + teleop
│   ├── .env.example             # Secret template
│   └── .env                     # Local secrets; gitignored
├── gps_rtk.sh                   # gps_mux + Point One Polaris supervisor
├── ros_start.sh                 # ROS bringup + teleop launcher
└── README.md
```

At runtime, `teleop.py` starts all robot-side subsystems in one process:

```text
cameras → IMU → GPS → motion → audio → lights → PTZ → Daily stream → recorder → UDP listeners → optional local dongle
```

Only `motion.py` uses ROS 2. Everything else talks directly to hardware or SDKs: HID relay, ONVIF PTZ, V4L2/RTSP cameras, UART IMU, UDP GPS, PulseAudio, Piper, ffmpeg, and Daily.

---

## What it does

- Drives the robot by publishing `geometry_msgs/Twist` to `/cmd_vel`.
- Streams one composed Daily video feed with selectable main camera, two optional PiP thumbnails, camera/speed overlays, and orbital-camera microphone audio.
- Records one camera to H.264 MP4 plus one JSONL telemetry row per video frame.
- Keeps video and telemetry aligned by `frame_index`: row `N` in `data.jsonl` corresponds to frame `N` in `video.mp4`.
- Reads WIT/JY901-style IMU frames directly from UART.
- Reads GPS/NMEA and UM982 `#ADRNAVA` solution metadata from a UDP feed produced by `gps_mux.py`.
- Supports RTK corrections through Point One Polaris while keeping teleop read-only access to the same GNSS receiver.
- Controls a 4-channel USB HID relay for headlights, strobe, parking lights, turn signals, talk blink, and all-lights choreography.
- Controls the ONVIF PTZ camera independently of drivetrain lock, with home capture and return-to-home.
- Provides PulseAudio output selection, volume control, Piper TTS, and WAV music playback.
- Arbitrates between a local USB dongle on the Jetson and the remote gamepad over Tailscale.

---

## Quick start

### 1. Copy secrets

```bash
cd ~/Revobots/aditya/aadi_scout
cp LAB/.env.example LAB/.env
nano LAB/.env
```

Required values:

```dotenv
DAILY_API_KEY=your_daily_api_key_here
PTZ_PASSWORD=your_ptz_password_here
POINTONE_API_KEY=your_point_one_key_here
POLARIS_UNIQUE_ID=your_polaris_unique_id_here
```

`teleop.py` reads `DAILY_API_KEY` and `PTZ_PASSWORD` through `LabConfig.load_secrets()`. `gps_rtk.sh` expects the Point One variables to be present in the shell environment, so source `.env` before running that script.

### 2. Review robot configuration

Edit `LAB/config.py` before running on a new robot. Important defaults:

| Area | Default |
|---|---|
| Motion topic | `/cmd_vel` |
| Motion publish rate | `50 Hz` |
| Motion watchdog | `0.30 s` |
| UDP motion port | `55999` |
| UDP events port | `57000` |
| UDP TTS port | `57001` |
| PTZ IP | `192.168.10.50` |
| PTZ ONVIF port | `8000` |
| Daily room | `https://revolabs.daily.co/scoutlab-pilot-cam` |
| IMU UART | `/dev/ttyCH341USB3 @ 9600` |
| GPS UDP feed | `127.0.0.1:57002` |
| Cache/session dir | `~/.cache/scout/lab` |
| Recording camera | `ai_front` |
| Recording format | `640x480 @ 15 fps`, H.264 MP4 + JSONL |

Configured cameras:

| Internal name | Source | Use |
|---|---|---|
| `orbital` | RTSP PTZ camera | Pilot/orbital view and mic source |
| `ai_front` | `/dev/video8` | Front AI camera and default main stream |
| `ai_back` | `/dev/video2` | Rear AI camera / right PiP |
| `floor` | `/dev/floor_cam` | Floor camera, available for recording if selected |

Gamepad camera aliases map `pilot → orbital`, `front → ai_front`, and `rear → ai_back`.

### 3. Run GPS/RTK supervisor, when RTK is needed

Use a separate terminal:

```bash
cd ~/Revobots/aditya/aadi_scout
set -a; source LAB/.env; set +a
./gps_rtk.sh
```

`gps_rtk.sh` starts `LAB/utils/gps_mux.py`, waits for `/tmp/scoutlab_gps_pty`, then launches the Point One Polaris serial client. If either child exits, the script kills the other child and exits cleanly so a restart returns to a known-good pair.

`gps_mux.py` owns the real GNSS serial device and exposes:

```text
/dev/ttyCH341USB2
  └── gps_mux.py owns the physical serial port
      ├── /tmp/scoutlab_gps_pty      # bidirectional PTY for Polaris NMEA/RTCM
      └── UDP 127.0.0.1:57002        # read-only NMEA feed for teleop GpsReader
```

Environment overrides supported by `gps_mux.py`:

| Variable | Default |
|---|---|
| `GPS_REAL_PORT` | `/dev/ttyCH341USB2` |
| `GPS_REAL_BAUD` | `115200` |
| `GPS_PTY_PATH` | `/tmp/scoutlab_gps_pty` |
| `GPS_UDP_HOST` | `127.0.0.1` |
| `GPS_UDP_PORT` | `57002` |

### 4. Start ROS bringup + teleop

Use another terminal:

```bash
cd ~/Revobots/aditya/aadi_scout
./ros_start.sh
```

`ros_start.sh` does the following:

1. Kills lingering `agv_pro_bringup`, `agv_pro_node`, and `lslidar_driver_node` processes.
2. Clears stale FastDDS shared-memory lock files from `/dev/shm/fastrtps*`.
3. Sources ROS 2 Humble and `~/agv_pro_ros2/install/local_setup.bash`.
4. Launches `agv_pro_bringup agv_pro_bringup.launch.py` in the background.
5. Waits 5 seconds and fails fast if bringup died.
6. Runs `python3 teleop.py` from the `LAB` directory.
7. On Ctrl-C or exit, cleans up ROS bringup and child nodes.

You can also run teleop manually after sourcing ROS:

```bash
cd ~/Revobots/aditya/aadi_scout
source /opt/ros/humble/setup.bash
source ~/agv_pro_ros2/install/local_setup.bash
python3 LAB/teleop.py
```

Optional CLI flags:

```bash
python3 LAB/teleop.py --env /path/to/.env
python3 LAB/teleop.py --no-local-dongle
```

---

## UDP protocol

`teleop.py` binds three UDP listeners.

| Port | Label | Purpose |
|---:|---|---|
| `55999` | motion | Driving, robot lock, PTZ head direction, camera switching, PTZ home actions, recording |
| `57000` | events | Lights, signals, talk blink, audio volume, music |
| `57001` | tts | Text-to-speech payloads |

### Motion/control port: `55999`

Accepted fields include:

| Field | Meaning |
|---|---|
| `lin_x`, `linx`, `linear_x` | Linear velocity command |
| `ang_z`, `angz`, `angular_z` | Angular velocity command; scaled by `ang_z_scale` in `motion.py` |
| `brake` | Brake input; active above `brake_threshold` |
| `robot_lock` or `lock` | Locks drivetrain; also disables lights/stream publishing and pauses recorder writes |
| `camera`, `cam`, `video_source` | Switches Daily main camera source |
| `head` | PTZ direction: `left`, `right`, `up`, `down`, `center` |
| `speed` | Speed-label change triggers PTZ home capture after the first value |
| `a` / `b`, or `button: 1/2` | A+B combo captures PTZ home |
| `button: 8` | Returns PTZ to captured home by default |
| `record: true` | Starts recording on rising edge |
| `record: false` | Stops recording on falling edge |

Example:

```json
{
  "lin_x": 0.4,
  "ang_z": -0.15,
  "robot_lock": false,
  "camera": "front",
  "head": "center",
  "record": true
}
```

### Events port: `57000`

| Event | Fields | Effect |
|---|---|---|
| `lights` | `headlights`, `strobe`, `parklights` | Steady relay state |
| `signals` | `left`, `right` | Timed turn-signal blink |
| `talk` | `duration` | All-light blink choreography |
| `audio` | `volume_pct` | PulseAudio default sink volume |
| `music` | `action: play`, `track` | Plays configured WAV track |

Examples:

```json
{"event":"audio", "volume_pct":75}
```

```json
{"event":"music", "action":"play", "track":1}
```

### TTS port: `57001`

```json
{"type":"stt", "text":"Hello from the robot"}
```

The text is queued into Piper TTS. Playback happens in a background thread so the command loop is not blocked.

---

## Command-source arbitration

There are two command sources:

| Source | Priority | Notes |
|---|---:|---|
| Local USB dongle | `100` | Highest priority, read through `evdev` on the Jetson |
| Remote gamepad | `200` | UDP packets over the network/Tailscale |

Lower priority number wins. If the active source goes silent for more than `source_activity_timeout_sec` (`1.0 s` by default), the other live source may take over.

The local dongle listener emits the same packet shape as the UDP gamepad, tags it internally as local, and routes it through the same dispatcher. Use `--no-local-dongle` to disable it.

---

## Recording

`LAB/record.py` records exactly one camera per session to H.264 MP4 and writes one telemetry row per encoded video frame.

Default output directory:

```text
~/.cache/scout/lab/session_YYYYMMDD_HHMMSS/
├── video.mp4
├── data.jsonl
└── session.json
```

### Recording controls

| Method | Action |
|---|---|
| Keyboard `r` | Toggle recording, if `pynput` is installed |
| UDP `{"record": true}` on port `55999` | Start recording |
| UDP `{"record": false}` on port `55999` | Stop recording |
| Ctrl-C | Stops active recorder before shutdown |

Recording is edge-triggered for UDP control, so repeated `record: true` packets do not restart the session.

### Recorder behavior

On each tick at `record_fps`:

1. Read the latest frame from `record_camera_name`.
2. Resize to `record_width × record_height` if needed.
3. Write raw BGR bytes into a long-lived `ffmpeg` process.
4. Build one flat telemetry row from motion state, IMU snapshot, and GPS snapshot.
5. Append that row to `data.jsonl`.
6. Increment `frame_index`.

When `robot_lock` is true, the recorder pauses frame consumption and JSONL writes. This keeps the training dataset focused on intentional driving.

### Encoder selection

`record_encoder_preference` is tried in order. The current default is:

```python
["libx264"]
```

`record.py` also contains ffmpeg command paths for `h264_nvenc` and `h264_v4l2m2m`, so those can be added back into the config preference list if the target Jetson image exposes them.

### `session.json`

Written when recording stops:

```json
{
  "session_dir": ".../session_YYYYMMDD_HHMMSS",
  "start_unix": 1780000000.0,
  "start_iso": "2026-06-04T20:00:00",
  "fps": 15,
  "frame_count": 1234,
  "duration_sec": 82.2667,
  "encoder": "libx264",
  "width": 640,
  "height": 480,
  "video": "video.mp4",
  "telemetry": "data.jsonl",
  "camera": "ai_front"
}
```

### `data.jsonl` row schema

Each row is a flat JSON object with fields like:

```json
{
  "frame_index": 0,
  "ts_unix": 1780000000.1234,
  "ts_capture": 12345.6789,
  "relative_time": 0.0,
  "linear_velocity": 0.4,
  "angular_velocity": -0.15,
  "robot_locked": false,
  "braking": false,
  "accelerometer_x": 0.01,
  "accelerometer_y": -0.02,
  "accelerometer_z": 1.0,
  "gyroscope_x": 0.0,
  "gyroscope_y": 0.0,
  "gyroscope_z": 0.1,
  "magnetometer_x": 100,
  "magnetometer_y": 50,
  "magnetometer_z": -20,
  "roll": 0.1,
  "pitch": -0.2,
  "yaw": 90.0,
  "gps_latitude": 37.0,
  "gps_longitude": -122.0,
  "gps_altitude": 12.3,
  "gps_fix": "RTK_FIXED",
  "gps_satellites": 16,
  "gps_hdop": 0.6,
  "gps_speed_kmh": 0.2,
  "orientation": 91.2,
  "gps_solution_status": "SOL_COMPUTED",
  "gps_position_type": "NARROW_INT"
}
```

The alignment guarantee is:

```text
data.jsonl line N  ==  video.mp4 frame N
```

---

## Subsystems

### `LAB/teleop.py`

Main entry point. It loads config/secrets, initializes `rclpy` once, starts every subsystem, binds UDP listeners, starts the optional local-dongle listener, and handles shutdown.

Startup order:

1. `LabConfig.load_secrets()`
2. `rclpy.init()` if available
3. `MultiCameraCapture.from_configs()`
4. `ImuReader.start()`
5. `GpsReader.start()`
6. `MotionController.start()`
7. `AudioController.start()`
8. `LightsController.start()`
9. `PtzController.start()`
10. `DailyStream.start()`
11. `SessionRecorder(...)`
12. Three `UdpListener` threads
13. Optional `LocalDongleListener`
14. Optional keyboard `r` listener

### `LAB/motion.py`

Publishes `Twist` to `/cmd_vel` at `motion_publish_hz`.

Safety behavior:

- Starts locked.
- Watchdog outputs zero if no fresh command arrives within `motion_watchdog_sec`.
- `robot_lock=True` forces zero output.
- `brake=True` forces zero output.
- `stop()` publishes three zero commands before shutdown.

### `LAB/cameras.py`

Each camera runs in its own background thread and stores only the latest frame. Reads are non-blocking:

```python
capture_ts, frame = cameras.read("ai_front")
```

RTSP sources use low-latency OpenCV/FFmpeg options and reconnect with backoff after drops. USB/V4L2 cameras are exclusive; a busy device is skipped and the rest of the system continues.

### `LAB/stream.py`

Uses the Daily Python SDK to publish:

- One virtual camera named `lab-virtual-cam`.
- Optional virtual microphone named `lab-virtual-mic`.
- Composed frames from the active main camera.
- Two PiP thumbnails when enabled.
- Optional speed, camera-name, and timestamp overlays.

When `robot_lock=True`, stream publishing is paused so the operator gets a clear lock-state indication.

### `LAB/lights.py`

Controls a 4-channel USB HID relay:

| Channel | Function |
|---:|---|
| 1 | Headlights |
| 2 | Strobe |
| 3 | Left halo + left tail |
| 4 | Right halo + right tail |

It supports three independent states:

- Steady lights.
- Turn-signal blink with timeout.
- All-channel blink for talk events and all-lights choreography.

`robot_lock=True` forces all relay channels off and ignores further light commands until unlocked.

### `LAB/ptz.py`

Controls the ONVIF PTZ camera. The PTZ subsystem is intentionally independent from drivetrain lock, so the operator can still look around while the robot is locked.

Supported command directions:

```text
left, right, up, down, center
```

Home behavior:

- First unlock can capture home.
- A+B combo captures home.
- Speed-label changes can capture home.
- Button `8` calls `goto_home()` by default.
- Position is dead-reckoned from issued velocity commands, not absolute camera feedback.

### `LAB/audio.py`

Handles:

- PulseAudio default sink/source auto-selection using configured name patterns.
- Startup volume.
- Non-blocking volume changes through `pactl`.
- Piper voice loading and queued TTS synthesis.
- WAV music playback and replacement through subprocess playback.

Configured music tracks:

| Track | File |
|---:|---|
| 1 | `REVOBOTS_Anthem_v1.wav` |
| 2 | `REVO_Track_old1.wav` |
| 3 | `REVO_Track_old2.wav` |

### `LAB/sensors.py`

Contains two readers.

`ImuReader`:

- Reads WIT/JY901 binary frames from `/dev/ttyCH341USB3` by default.
- Supports accelerometer, gyroscope, roll/pitch/yaw, magnetometer, and optional quaternion frames.
- Auto-reconnects with exponential backoff.

`GpsReader`:

- Binds UDP `127.0.0.1:57002` by default.
- Parses NMEA `GGA`, `RMC`, `VTG`, and `HDT`.
- Parses UM982 `#ADRNAVA` extensions for solution status and position type.
- Uses true heading from `HDT` as `orientation` when available; otherwise falls back to course-over-ground.

### `LAB/utils/gps_mux.py`

Owns the GNSS receiver serial port and shares it safely between Polaris and teleop:

- Receiver NMEA → PTY and UDP fan-out.
- Polaris RTCM from PTY → receiver.
- USB reconnect handling.
- Stable PTY symlink across reconnects.

### `gps_rtk.sh`

Supervisor for `gps_mux.py` and the Point One Polaris serial client. It uses a lock file at `/tmp/scoutlab_gps_rtk.lock` to prevent multiple instances.

### `ros_start.sh`

Robot-side launch script for ROS bringup and teleop. It cleans stale processes/locks before startup and cleans up ROS nodes on exit.

---

## Configuration reference

### UDP and arbitration

| Field | Default | Purpose |
|---|---:|---|
| `udp_listen_ip` | `0.0.0.0` | Bind interface |
| `udp_motion_port` | `55999` | Motion/control packets |
| `udp_events_port` | `57000` | Events/audio/lights packets |
| `udp_tts_port` | `57001` | TTS packets |
| `local_dongle_priority` | `100` | Local command priority |
| `remote_gamepad_priority` | `200` | Remote command priority |
| `source_activity_timeout_sec` | `1.0` | Source silence timeout |

### Motion

| Field | Default |
|---|---:|
| `cmd_vel_topic` | `/cmd_vel` |
| `motion_publish_hz` | `50` |
| `motion_watchdog_sec` | `0.30` |
| `ang_z_scale` | `0.20` |
| `brake_threshold` | `0.20` |

### PTZ

| Field | Default |
|---|---:|
| `ptz_ip` | `192.168.10.50` |
| `ptz_port` | `8000` |
| `ptz_user` | `revolabs` |
| `ptz_pan_speed` | `0.65` |
| `ptz_tilt_speed` | `0.55` |
| `ptz_loop_hz` | `25.0` |
| `ptz_deadband_sec` | `0.05` |
| `ptz_stop_after_sec` | `0.15` |
| `ptz_home_button` | `8` |

### Streaming and overlays

| Field | Default |
|---|---|
| `daily_room_url` | `https://revolabs.daily.co/scoutlab-pilot-cam` |
| `daily_room_name` | `scoutlab-pilot-cam` |
| `stream_width` / `stream_height` | `640 / 480` |
| `stream_fps` | `15` |
| `initial_main_source` | `ai_front` |
| `pip_enabled` | `True` |
| `pip_left_source` | `orbital` |
| `pip_right_source` | `ai_back` |
| `pip_width` / `pip_height` | `192 / 144` |
| `overlay_speed_badge` | `True` |
| `overlay_camera_name` | `True` |
| `overlay_timestamp` | `False` |

### Recording

| Field | Default |
|---|---|
| `cache_dir` | `~/.cache/scout/lab` |
| `record_camera_name` | `ai_front` |
| `record_width` / `record_height` | `640 / 480` |
| `record_fps` | `15` |
| `record_video_bitrate` | `1500k` |
| `record_encoder_preference` | `["libx264"]` |

### Sensors

| Field | Default |
|---|---|
| `imu_port_hint` | `/dev/ttyCH341USB3` |
| `imu_baud` | `9600` |
| `gps_udp_host` | `127.0.0.1` |
| `gps_udp_port` | `57002` |

---

## Dependencies

System packages and runtime tools commonly needed on the Jetson:

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  ffmpeg \
  pulseaudio-utils \
  ros-humble-desktop \
  v4l-utils
```

Python packages used by the current codebase include:

```bash
pip3 install \
  numpy \
  opencv-python \
  pyserial \
  requests \
  evdev \
  pynput \
  hid \
  onvif-zeep \
  piper-tts \
  daily-python
```

ROS-side dependencies are provided by the robot workspace, especially `agv_pro_bringup`, `geometry_msgs`, and `rclpy`.

Hardware/SDK dependencies:

- 4-channel USB HID relay board: vendor `0x16c0`, product `0x05df`.
- ONVIF PTZ camera reachable at `ptz_ip:ptz_port`.
- Daily Python SDK and valid Daily room/API key.
- Piper model at `piper_model` path.
- Point One Polaris serial client at `/home/elephant/Revobots/polaris/build/examples/serial_port_client` for RTK.

---

## Verification

### Check teleop listeners

When `teleop.py` starts, expect logs similar to:

```text
[teleop] ports = motion:55999 events:57000 tts:57001
[teleop] UDP listener motion on 0.0.0.0:55999
[teleop] UDP listener events on 0.0.0.0:57000
[teleop] UDP listener tts on 0.0.0.0:57001
[teleop] ready — 'r' to toggle recording, Ctrl-C to quit
```

### Check GPS/RTK

After `gps_rtk.sh` has been running outdoors for a few minutes, inspect the latest JSONL row:

```bash
tail -1 ~/.cache/scout/lab/session_<latest>/data.jsonl | python3 -m json.tool | \
  grep -E 'gps_fix|gps_solution_status|gps_position_type|gps_satellites|gps_hdop|orientation'
```

Healthy RTK output should eventually include values like:

```json
"gps_fix": "RTK_FIXED",
"gps_solution_status": "SOL_COMPUTED",
"gps_position_type": "NARROW_INT",
"gps_satellites": 16,
"gps_hdop": 0.6
```

### Check recording alignment

```bash
SESSION=~/.cache/scout/lab/session_<latest>
wc -l "$SESSION/data.jsonl"
ffprobe -v error -select_streams v:0 -count_frames \
  -show_entries stream=nb_read_frames,r_frame_rate \
  -of default=nokey=1:noprint_wrappers=1 "$SESSION/video.mp4"
cat "$SESSION/session.json"
```

The JSONL line count, `session.json.frame_count`, and MP4 frame count should match.

---

## Notes and known behavior

- `record.py` now writes `video.mp4`, `data.jsonl`, and `session.json`; it no longer writes per-camera JPEG folders or uses an `image_writer.py` pipeline.
- `sensors.py` now reads IMU directly from UART and GPS from the `gps_mux.py` UDP fan-out; it no longer tails journals or depends on ROS GPS topics.
- `teleop.py` currently has a duplicated shutdown log/recorder-stop block in the `finally` path. It is harmless because `recorder.stop()` is idempotent when inactive, but it can print `shutting down…` twice.
- `LabConfig.load_secrets()` currently loads only `DAILY_API_KEY` and `PTZ_PASSWORD`. `POINTONE_API_KEY` and `POLARIS_UNIQUE_ID` are still required by `gps_rtk.sh`, so source `LAB/.env` before launching GPS/RTK.
- `record_encoder_preference` defaults to `libx264`; hardware encoders are available in `record.py` but not enabled in config unless you add them.
- If `pynput` is not installed or the process has no interactive display/session, the keyboard `r` toggle is disabled. UDP recording control still works.

---

## Safe shutdown

Stop GPS/RTK and teleop with Ctrl-C in their respective terminals.

- `teleop.py` stops active recording first, then stops UDP listeners, local dongle, stream, PTZ, lights, audio, motion, sensors, cameras, and finally shuts down `rclpy`.
- `ros_start.sh` traps exit and kills ROS bringup child processes.
- `gps_rtk.sh` traps exit and kills both the mux and Polaris client together.
