# REVO Scout LAB

Standalone controller and dataset recorder for the Segway Scout XT.  
One command starts everything — teleop, Daily streaming, sensors, lights, PTZ, audio, and optional recording.

```bash
./ros_start.sh&
python LAB/teleop.py
```

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [File Overview](#file-overview)
3. [teleop.py — Main Script](#telepopy--main-script)
4. [record.py — Recording](#recordpy--recording)
5. [config.py — All Settings](#configpy--all-settings)
6. [common.py — Shared Utilities](#commonpy--shared-utilities)
7. [cameras.py](#cameraspy)
8. [motion.py](#motionpy)
9. [lights.py](#lightspy)
10. [ptz.py](#ptzpy)
11. [stream.py](#streampy)
12. [audio.py](#audiopy)
13. [sensors.py](#sensorspy)
14. [image_writer.py](#image_writerpy)
15. [Dataset Output](#dataset-output)
16. [Dependencies](#dependencies)

---

## Quick Start

**1. Copy and fill in secrets**
```bash
cp LAB/.env.example LAB/.env
# edit LAB/.env — add DAILY_API_KEY and PTZ_PASSWORD
```

**2. Review config**  
Open `LAB/config.py` and verify camera RTSP URLs, device paths, and IP addresses match your robot.

**3. Run**
```bash
source /opt/ros/humble/setup.bash
source ~/agv_pro_ros2/install/local_setup.bash
python LAB/teleop.py
```

**4. Record**

| Method | Action |
|--------|--------|
| Keyboard `r` | Toggle recording on / off |
| UDP `"record": true` | Start recording |
| UDP `"record": false` | Stop recording |
| `Ctrl-C` | Quit and save |

---

## File Overview

```
LAB/
  teleop.py        Main entry point — starts everything, dispatches UDP commands
  record.py        SessionRecorder — writes data.json + JPEG frames to disk
  config.py        Every tunable value in one @dataclass; only secrets come from .env
  common.py        Shared helpers: log(), truthy(), first_float(), now_mono()
  cameras.py       Background camera capture (RTSP + V4L2) shared by stream + recorder
  motion.py        Velocity commands → ROS2 /cmd_vel at 50 Hz with watchdog
  lights.py        4-channel HID relay: steady, turn-signal blink, all-blink
  ptz.py           ONVIF PTZ with dead-reckoning and home capture/return
  stream.py        Daily.co stream: main camera + PiP thumbnails + badges + mic
  audio.py         PulseAudio auto-select, volume, music playback, Piper TTS
  sensors.py       IMU journal reader + GPS ROS2 subscriber
  image_writer.py  Async JPEG writer thread pool (used by recorder)
  .env             Secrets only — DAILY_API_KEY and PTZ_PASSWORD
  .env.example     Template for .env
```

---

## teleop.py — Main Script

The single entry point. Starts all subsystems and dispatches every UDP packet to the right handler.

### Startup sequence

`main()` runs these steps in order:

1. `LabConfig.load_secrets()` — reads `config.py` defaults and injects secrets from `.env`
2. `MultiCameraCapture.from_configs()` — opens all cameras in background threads
3. `MotionController.start()` — creates ROS2 node and begins 50 Hz publish loop
4. `LightsController.start()` — opens HID relay board, starts blink timer thread
5. `PtzController.start()` — connects to ONVIF camera, starts control loop
6. `DailyStream.start()` — fetches token, joins Daily room, starts stream + mic
7. `AudioController.start()` — auto-selects PulseAudio device, loads Piper model, starts TTS worker
8. `ImuReader.start()` — tails IMU service journal in background thread
9. `GpsReader.start()` — creates ROS2 node and subscribes to GPS + heading topics
10. `UdpCommandListener.start()` — binds UDP port 55999 and begins receiving gamepad packets
11. `start_keyboard_listener()` — `pynput` listener for `r` key

### `UdpCommandListener`

Daemon thread that binds `udp_listen_ip:udp_listen_port` (default `0.0.0.0:55999`).  
For each packet received: JSON-parses it and calls `on_command(pkt)`.  
Socket timeout is 0.2 s so the thread wakes regularly to check the stop event.

> **Note:** `config.py` defines three separate ports (`udp_motion_port=55999`, `udp_events_port=57000`, `udp_tts_port=57001`) matching the original distributor layout. The current `UdpCommandListener` binds only the motion port. Binding the remaining two ports (lights/audio events and TTS) is the next step.

### `on_command(pkt)` — the dispatcher

Called for every received UDP packet. Reads the same JSON field names as the original services.

| Payload field | Routed to |
|---------------|-----------|
| `lin_x` / `ang_z` / `brake` / `robot_lock` | `motion.command()` |
| `robot_lock` | `lights.set_robot_lock()`, `stream.set_robot_lock()` |
| `event="lights"` + `headlights` / `strobe` / `parklights` | `lights.command()` |
| `event="signals"` + `left` / `right` | `lights.command()` |
| `event="talk"` + `duration` | `lights.command()` (all-blink), `audio.speak()` |
| `event="audio"` + `volume_pct` | `audio.set_volume()` |
| `event="music"` + `track` | `audio.play_music()` |
| `head` | `ptz.command()` |
| `button` == `ptz_home_button` (8) | `ptz.goto_home()` |
| `speed` field present | `ptz.capture_home()` |
| `camera` / `cam` / `video_source` | `stream.switch_source()` |
| `record=true` | `start_recording()` |
| `record=false` | `stop_recording()` |

### Recording functions

**`start_recording()`** — creates a fresh `SessionRecorder` pointed at `cfg.cache_dir`. Guards with a lock so double-press is safe.

**`stop_recording()`** — sets `recording=False`, then calls `recorder.stop()` which flushes all pending writes before returning.

**`toggle_recording()`** — called by the `r` key listener; calls start or stop depending on current state.

### Recording loop

Runs in `main()` at `cfg.record_fps` (default 15 Hz). On each tick when recording is active:

```
lin_x, ang_z  ← motion.state()
imu_data      ← imu.get()
gps_data      ← gps.get()
camera_frames ← cameras.read_all()
recorder.save_frame(...)
```

### Shutdown

`Ctrl-C` / `SIGTERM` → `on_shutdown()` sets `running=False`. The loop exits and calls `.stop()` on every subsystem in order, ensuring the recorder flushes before the process exits.

### CLI flags

```
--env /path/to/.env    Override default LAB/.env location
--fps 10               Override record_fps at runtime
```

---

## record.py — Recording

`SessionRecorder` manages one continuous session from press-`r` to press-`r`.

### `__init__(base_dir, camera_names, jpeg_quality)`

Creates:
```
{base_dir}/session_20260602_143022/
    data.json
    image/
        orbital/
        floor/
        ai_front/
        …
```
- Names the folder to the second — no collisions between sessions
- Opens `data.json` in write mode
- Creates one `image/<name>/` subdirectory per camera upfront
- Starts an `AsyncImageWriter` with `cfg.image_writer_workers` threads

### `save_frame(linear_velocity, angular_velocity, imu, gps, camera_frames)`

Called once per tick by the recording loop.

1. Reads and increments `_frame_index`
2. Builds the full data row (see [Dataset Output](#dataset-output))
3. Writes one JSON line to `data.json` and `flush()`es immediately — partial sessions are readable even if the process crashes
4. For each camera in `camera_frames`, calls `_writer.save(frame, path)` — non-blocking, returns immediately

### `stop()`

1. Calls `_writer.flush()` — blocks until all queued JPEGs are on disk
2. Calls `_writer.stop()` — sends shutdown sentinels to each writer thread
3. Closes `data.json`

---

## config.py — All Settings

`LabConfig` is a `@dataclass`. Every field has a default value — edit the file directly to retune the robot. Secrets are the only things that come from outside.

### `CameraConfig` dataclass

One physical camera source.

| Field | Description |
|-------|-------------|
| `name` | Internal identifier, e.g. `"orbital"`, `"floor"`. Used as image folder name. |
| `source` | RTSP URL (`rtsp://...`) or V4L2 path (`/dev/floor_cam`) |
| `width`, `height`, `fps` | Capture resolution and frame rate |
| `rtsp_transport` | `"tcp"` or `"udp"` — validated on init |
| `is_rtsp` (property) | `True` when source starts with `rtsp://` |

### UDP listener ports

| Field | Default | What listens here |
|-------|---------|-------------------|
| `udp_motion_port` | `55999` | `lin_x`, `ang_z`, `head`, `camera`, `button`, `robot_lock` |
| `udp_events_port` | `57000` | `lights`, `signals`, `audio`, `talk`, `music` |
| `udp_tts_port` | `57001` | `type:"stt"`, `text` (TTS text) |
| `udp_listen_ip` | `"0.0.0.0"` | Bind interface for all listeners |

These match what the gamepad operator code sends to `--robot ELEPHANT`.

### Source priority arbitration

| Field | Default | Description |
|-------|---------|-------------|
| `local_dongle_priority` | `100` | Lower number = higher priority |
| `remote_gamepad_priority` | `200` | Remote yields to local dongle |
| `source_activity_timeout_sec` | `1.0` | Port silent this long → other source takes over |

### Motion

| Field | Default | Description |
|-------|---------|-------------|
| `cmd_vel_topic` | `"/cmd_vel"` | ROS2 topic |
| `motion_publish_hz` | `50` | Publish rate |
| `motion_watchdog_sec` | `0.30` | Stop robot if no command for this long |
| `ang_z_scale` | `0.20` | Turning attenuation factor |
| `brake_threshold` | `0.20` | `brake` value above this triggers stop |

### PTZ

| Field | Default | Description |
|-------|---------|-------------|
| `ptz_ip` | `192.168.10.50` | Camera IP |
| `ptz_port` | `8000` | ONVIF port |
| `ptz_user` | `"revolabs"` | ONVIF username |
| `ptz_pan_speed` | `0.65` | Pan velocity (−1.0 to 1.0) |
| `ptz_tilt_speed` | `0.55` | Tilt velocity |
| `ptz_loop_hz` | `25.0` | PTZ control loop rate |
| `ptz_deadband_sec` | `0.05` | Min interval before re-issuing same command |
| `ptz_stop_after_sec` | `0.15` | Auto-stop if no command arrives |
| `ptz_home_button` | `8` | Gamepad button that returns to home |

### Lights

| Field | Default | Description |
|-------|---------|-------------|
| `blink_period_sec` | `0.40` | Full on+off cycle for turn signals |
| `signal_timeout_sec` | `20.0` | Auto-cancel turn signal after this |
| `talk_default_duration` | `7.0` | All-blink duration for talk events |
| `all_lights_cooldown_sec` | `5.0` | Absorbs 10× repeat from gamepad sender |
| `all_lights_blink_sec` | `5.0` | Blink duration before latching all-on |

### Audio

| Field | Default | Description |
|-------|---------|-------------|
| `piper_model` | `~/Revobots/piper/voices/en_GB-…` | Full path to `.onnx` TTS model |
| `piper_speaker_id` | `None` | Speaker index for multi-speaker models |
| `music_dir` | `~/Revobots/Audio` | Directory containing WAV music files |
| `music_tracks` | `{1: "…Anthem…", 2: …, 3: …}` | Track number → filename mapping |
| `startup_volume_pct` | `100` | Volume set on `start()` |
| `preferred_sink_patterns` | `["ugreen", "emeet", …]` | Ordered list of sink name fragments to prefer |
| `preferred_source_patterns` | `["ugreen", "emeet", …]` | Same for mic input |

### Cameras

`cameras` is a list of `CameraConfig` entries. Cameras are referenced by `name` everywhere else. Camera names used in recordings become image folder names.

`camera_name_aliases` maps incoming UDP names to internal names:
```python
{"pilot": "orbital", "front": "ai_front", "rear": "ai_back", …}
```
The gamepad can send `"camera": "pilot"` and the stream correctly switches to `"orbital"`.

### Daily streaming

| Field | Default | Description |
|-------|---------|-------------|
| `daily_room_url` | `https://revobots.daily.co/scout-lab` | Room join URL |
| `daily_room_name` | `"scout-lab"` | Used for token creation |
| `stream_width`, `stream_height` | `640, 480` | Output resolution |
| `stream_fps` | `15` | Target stream frame rate |
| `initial_main_source` | `"ai_front"` | Camera shown on join |

### Picture-in-Picture

| Field | Default | Description |
|-------|---------|-------------|
| `pip_enabled` | `True` | Composite thumbnails onto main stream |
| `pip_left_source` | `"orbital"` | Camera name for left thumbnail |
| `pip_right_source` | `"ai_back"` | Camera name for right thumbnail |
| `pip_width`, `pip_height` | `192, 144` | Thumbnail dimensions in pixels |
| `pip_margin` | `12` | Pixels from frame edge |
| `pip_gap` | `8` | Gap between thumbnails (unused for left+right) |
| `pip_stale_sec` | `0.60` | Drop thumbnail if frame is older than this |
| `pip_show_label` | `True` | Draw `thumb:<name>` label in corner |
| `overlay_speed_badge` | `True` | Draw speed / LOCKED / BRAKE text |
| `overlay_camera_name` | `True` | Draw current source name |
| `overlay_timestamp` | `False` | Draw HH:MM:SS clock |

### Mic

| Field | Default | Description |
|-------|---------|-------------|
| `mic_rtsp_url` | orbital sub-stream URL | RTSP source for audio |
| `mic_rtsp_transport` | `"tcp"` | Transport for mic RTSP |
| `mic_sample_rate` | `16000` | PCM sample rate (Hz) |
| `mic_channels` | `1` | Mono |
| `mic_frame_ms` | `5` | PCM chunk size pushed to Daily |

### Sensors (UART)

| Field | Default | Description |
|-------|---------|-------------|
| `imu_port_hint` | `"/dev/ttyCH341USB3"` | Preferred serial port for IMU |
| `imu_baud` | `9600` | IMU baud rate |
| `gps_port_hint` | `"/dev/ttyCH341USB2"` | Preferred serial port for GPS |
| `gps_baud` | `115200` | GPS baud rate |

### Recording

| Field | Default | Description |
|-------|---------|-------------|
| `cache_dir` | `~/.cache/scout/lab` | Root folder for all sessions |
| `record_camera_name` | `"floor"` | Primary camera saved per session |
| `record_fps` | `15` | Dataset frame rate |
| `record_video_bitrate` | `"1500k"` | FFmpeg `-b:v` for video encoding |
| `record_encoder_preference` | `["h264_nvenc", "h264_v4l2m2m", "libx264"]` | Try Jetson HW encoder first |

### Local dongle

| Field | Default | Description |
|-------|---------|-------------|
| `local_dongle_enabled` | `True` | Enable evdev gamepad on the robot |
| `local_dongle_name_hints` | `["8bitdo", "tgz", "cx 2.4g", …]` | Name fragments identifying the gamepad |

### Secrets

Only these two fields come from `.env`:

| Field | `.env` key | Description |
|-------|-----------|-------------|
| `daily_api_key` | `DAILY_API_KEY` | Daily.co REST API key |
| `ptz_password` | `PTZ_PASSWORD` | ONVIF password for the PTZ camera |

### `LabConfig.load_secrets(env_file=None)`

The only constructor used. Creates a `LabConfig` instance from defaults, then reads `LAB/.env` (or the path you pass) and injects the two secrets. Everything else is defined in the class.

---

## common.py — Shared Utilities

Imported by every module. Kept minimal — only things used in more than one place.

| Function | Description |
|----------|-------------|
| `log(tag, msg)` | Prints `[tag] msg` to stdout with `flush=True`. Used instead of `print()` everywhere. |
| `truthy(value)` | Permissive bool coercion for UDP JSON values. Accepts `bool`, `int/float`, and strings `"1"/"true"/"yes"/"on"/"pressed"/"down"`. |
| `first_float(pkt, keys, default)` | Returns the first value from `pkt` matching any key in `keys`, parsed as `float`. Handles aliased field names cleanly. |
| `first_int(pkt, keys, default)` | Same but returns `int`. |
| `now_mono()` | `time.monotonic()` — for measuring intervals. Never goes backward. |
| `now_unix()` | `time.time()` — wall clock for dataset timestamps and human-readable output. |

---

## cameras.py

Single background thread per camera. Both `stream.py` and `record.py` read from the same objects — one connection per camera regardless of how many consumers exist.

### `CameraCapture`

| Method | Description |
|--------|-------------|
| `start()` | Probe-opens the source to verify it's reachable, then starts the reader thread. Returns `True` on success. Skipped cameras are logged and absent from the collection. |
| `read_latest()` | Non-blocking. Returns `(timestamp, frame)` or `(None, None)`. |
| `stop()` | Sets stop event and joins thread (2 s timeout). |
| `_open_capture()` | Builds a `cv2.VideoCapture` with low-latency FFmpeg env options for RTSP, V4L2 backend for USB. Sets 1-frame kernel buffer. |
| `_run()` | Continuous read loop. On disconnect logs the event and reconnects with exponential backoff (1 s → 2 s → max 10 s). |

### `MultiCameraCapture`

| Method | Description |
|--------|-------------|
| `from_configs(configs)` | Class method. Starts one `CameraCapture` per config; skips unreachable ones. |
| `has(name)` | Returns `True` if a camera with this name successfully opened. Used by `stream.py` before reading thumbnails. |
| `read(name)` | Returns `(timestamp, frame)` for one named camera. |
| `read_all()` | Returns `{name: (timestamp, frame)}` for every camera that has a current frame. |
| `names()` | List of successfully opened camera names. |
| `stop_all()` | Stops all cameras and clears the collection. |

---

## motion.py

Publishes `geometry_msgs/Twist` to `/cmd_vel` at 50 Hz. Three independent safety gates ensure the robot stops safely.

### `MotionController`

**`start()`**  
Requires `rclpy.init()` to have been called beforehand (the orchestrator does this). Creates a ROS2 node, a `SingleThreadedExecutor` (spun in its own thread so callbacks don't block the publisher), and starts `_pub_thread`.

**`command(lin_x, ang_z, locked, braking)`**  
Stores latest velocity and resets the watchdog timer. Called from `on_command()` in `teleop.py` on every motion packet. Thread-safe.

**`state() → (lin_x, ang_z, locked, braking)`**  
Thread-safe read of current motion state. Used by `stream.py` to draw the speed badge and by the recording loop to capture the action.

**`_compute_output() → (lin_x, ang_z)`**  
Applies three gates before each publish tick:
1. **Watchdog** — if no `command()` arrived within `motion_watchdog_sec` (0.3 s), output `(0, 0)`
2. **robot_lock** — if `locked=True`, output `(0, 0)`
3. **Brake** — if `braking=True`, output `(0, 0)`

`ang_z` is also multiplied by `ang_z_scale` (0.20) to attenuate turning.

**`stop()`**  
Publishes 3× zero-velocity at 20 ms intervals before destroying the node — ensures the robot receives a stop command even if the last packet was lost.

---

## lights.py

Controls a 4-channel USB HID relay board (`vid=0x16c0 pid=0x05df`).

Channel map: `1=headlights`, `2=strobe`, `3=halo_left`, `4=halo_right`

Three animation modes managed by a single blink thread. Precedence: **all-blink > turn signal > steady**.

### `LightsController`

**`start()`** — opens HID device, calls `all_off()`, starts blink thread.

**`set_robot_lock(locked)`** — when `True`, clears all state flags and calls `_apply_all_off()`. Subsequent `command()` calls are ignored until unlocked. Called from `on_command()` when `robot_lock` changes.

**`command(pkt)`** — dispatches to one of three event handlers based on `pkt["event"]`:
- `"lights"` → `_handle_lights_event()`: sets `headlights`, `strobe`, `parklights` steady state. If all three turn ON simultaneously, triggers the **all-blink combo** (blink all channels for `all_lights_blink_sec`, then latch everything steady-on). A cooldown of `all_lights_cooldown_sec` absorbs the gamepad's 10× repeat.
- `"signals"` → `_handle_signals_event()`: sets `_left_until` / `_right_until` timestamps from `now + signal_timeout_sec`. Setting a signal to false clears it immediately.
- `"talk"` → `_handle_talk_event()`: sets `_all_blink_until` for `duration` seconds. Fades back to steady state (not all-on) when it expires.

**`_blink_loop()`** — ticks every `blink_half` (0.20 s). Decides per tick:
- If `_all_blink_until` is active: all four channels toggle together
- Else if either signal is active: headlights/strobe held steady, halos blink per-channel
- Else: assert steady state each tick (cheap, idempotent)
- On all-blink expiry with `_all_blink_then_on=True`: latches all four channels on in the steady state

**`_write_relay(channel, on)`** — sends 3-byte HID command (`0xFF`=on, `0xFD`=off). Auto-reconnects on transient USB errors (up to 2 attempts, rate-limited log).

---

## ptz.py

ONVIF continuous-move PTZ with dead-reckoning position tracking and home capture/return.

PTZ motion is **independent of drivetrain `robot_lock`** — the operator can still look around while the robot is safety-stopped.

### `PtzController`

**`start()`** — connects via `onvif-zeep`, gets the first media profile token, sends an initial `Stop`, and starts `_loop()`.

**`command(head)`** — sets `_desired` direction (`"left"/"right"/"up"/"down"/"center"`). Resets the stop watchdog timer.

**`set_ptz_unlock_state(unlocked)`** — independent PTZ-only lock. On first unlock, captures the current dead-reckoned position as `_origin`. On lock, stops motion.

**`capture_home()`** — explicitly marks the current position as home. Called on A+B combo or speed-cycle events.

**`goto_home()`** — starts driving back toward stored home. Sets `_returning_pan` / `_returning_tilt` flags which the control loop uses to override operator direction until within `return_deadband`.

**`_loop()`** — ticks at `ptz_loop_hz` (25 Hz). Each tick:
1. Integrates `dt` to update dead-reckoned pan/tilt position
2. Checks if a return-to-home is active and within deadband (cancels it if so)
3. Decides the ONVIF command: `return_pan` → `return_tilt` → operator direction → center
4. Calls `_send_with_deadband()` — only re-issues `ContinuousMove` if the command changed or `deadband_sec` has elapsed

---

## stream.py

Publishes camera + microphone to a Daily.co room. Composes each outgoing frame from a main source, two PiP thumbnails, and overlaid badges.

### Frame composition per tick (`_compose_frame`)

1. Resize main frame to `stream_width × stream_height`
2. If `pip_enabled`: call `_overlay_thumbnail()` for `pip_left_source` and `pip_right_source`
3. If `overlay_camera_name`: draw source name top-left
4. If `overlay_speed_badge` and `motion_state_fn` provided: draw speed / LOCKED / BRAKE bottom-left
5. If `overlay_timestamp`: draw HH:MM:SS
6. `_push_rgb()`: resize if needed, convert BGR→RGB, call `cam_device.write_frame()`

### Robot lock behaviour

When `set_robot_lock(True)` is called, `_stream_loop()` holds the **last published frame** frozen (operator sees the last image, not black). Audio is **drained but not pushed** — the mic reader reads PCM from FFmpeg but does not call `write_frames()`, so Daily doesn't bill bandwidth for silence.

### `switch_source(name)`

Resolves `name` through `_aliases` (e.g. `"pilot"` → `"orbital"`) then checks `cameras.has(target)` before updating `_current_source`. Logs the switch. Silent no-op for unknown names.

### `DailyStream.__init__` parameters

All parameters come directly from `LabConfig` in the orchestrator. Notable ones:
- `motion_state_fn` — a zero-argument callable returning `(lin_x, ang_z, locked, braking)` used by the speed badge. Pass `motion.state` from `teleop.py`.
- `name_aliases` — the `camera_name_aliases` dict from config
- `pip_left_source`, `pip_right_source` — which camera names to use as thumbnails

### `_overlay_thumbnail(frame, src, position)`

Reads `cameras.read(src)`, checks the frame age against `pip_stale_sec`, resizes to `pip_width × pip_height`, composites onto the main frame at the top corner, draws a white border, and optionally labels with `"thumb:<name>"`.

---

## audio.py

All audio operations are fire-and-forget. The command loop is never blocked.

### Startup (`start()`)

1. `_init_pulseaudio_defaults()` — calls `pactl list short sinks/sources`, pattern-matches against `preferred_sink_patterns` and `preferred_source_patterns` to find the UGREEN/EMEET USB device, sets it as default, unmutes it
2. `set_volume(startup_volume_pct)` — fires a background thread to run `pactl set-sink-volume`
3. `_load_piper()` — loads the Piper voice with `PiperVoice.load(model_path)` into `self._voice` (~250 MB RAM, ~2 s on first call). If the model path is empty or the file is missing, logs a warning and TTS is silently disabled
4. `_tts_thread.start()` — begins the TTS worker

### `set_volume(pct)`

Clamps to 0–150 (pactl supports software boost above 100%). Runs `pactl set-sink-mute @DEFAULT_SINK@ 0` then `pactl set-sink-volume` in a daemon thread. Returns immediately.

### `speak(text)`

Enqueues text into `_tts_queue` (capacity 4). Silently dropped if the queue is full or the Piper model failed to load.

### `play_music(track_num)` / `stop_music()`

Looks up `track_num` in `music_tracks`, builds an absolute path from `music_dir`, kills any running music subprocess, then starts a new one. Player command is auto-detected at each call: tries `paplay`, `pw-play`, `ffplay`, `aplay` via `which`.

### TTS synthesis (`_synthesize_and_play`)

Synthesizes into an **in-memory WAV buffer** (no CLI piper subprocess — the `PiperVoice` Python API is called directly). Writes the buffer to a temp file, plays it with `_play_wav()`, then deletes the temp file in a `finally` block. An empty WAV (≤ 44 bytes header) is caught before playback.

---

## sensors.py

### `ImuReader`

Reads IMU data by following the systemd journal of `revo_scoutlab_revo_imu.service`.

- `start()` — launches `_read_journal()` in a daemon thread
- `_read_journal()` — runs `journalctl -u <unit> -f -n 0 --no-pager -o cat` as a subprocess and iterates stdout. Auto-restarts if the subprocess dies.
- `_parse_line(line)` — applies a compiled regex matching the IMU summary format: `ACC[g]=(x,y,z)  GYR[dps]=(x,y,z)  MAG[raw]=(x,y,z)  RPY[deg]=(roll,pitch,yaw)`. Returns 12 floats keyed as `accelerometer_x/y/z`, `gyroscope_x/y/z`, `magnetometer_x/y/z`, `roll`, `pitch`, `yaw`. Returns `None` for non-matching lines.
- `get()` — thread-safe copy of the latest parsed dict. Returns `{}` until the first valid line.
- `stop()` — sets stop event and terminates the journalctl subprocess.

> `config.py` has `imu_port_hint` / `imu_baud` for future direct UART reading. The journal approach used here works as long as the IMU service is running.

### `GpsReader`

ROS2 subscriber for `sensor_msgs/NavSatFix` and heading. Gracefully disabled if `rclpy` is not installed.

- `start()` — tries `import rclpy`; starts `_subscribe()` thread if available, otherwise logs a warning
- `_subscribe()` — initialises a ROS2 node, waits up to 5 s for `gps_topic` to appear, then creates subscriptions for both GPS and heading topics
- `_on_gps(msg)` — extracts `latitude`, `longitude`, `altitude`, `status.status`, `status.service`, `position_covariance` (list of 9), `position_covariance_type` from `NavSatFix`
- `_on_heading(msg)` — reads `msg.data` → stores as `"orientation"`
- `get()` — thread-safe copy of the latest GPS + heading dict

---

## image_writer.py

Decouples JPEG encoding and disk I/O from the recording loop so `save_frame()` never blocks.

### `AsyncImageWriter`

| Method | Description |
|--------|-------------|
| `__init__(jpeg_quality, num_workers)` | Creates a bounded queue (512 slots) and starts `num_workers` daemon threads. |
| `save(frame, path)` | Enqueues `(frame, path)`. Returns immediately. Frame is silently dropped if the queue is full. |
| `_worker()` | Dequeues frames and calls `cv2.imwrite()` with JPEG quality setting. Creates parent directories on first write. Exits on `None` sentinel. |
| `flush()` | Calls `queue.join()` — blocks until every enqueued item has been processed. Called by `SessionRecorder.stop()`. |
| `stop()` | Sends one `None` sentinel per worker thread, then joins all (10 s timeout). |

---

## Dataset Output

```
~/.cache/scout/lab/
  session_20260602_143022/
    data.json
    image/
      orbital/
        frame_000000.jpg
        …
      floor/
        frame_000000.jpg
        …
```

`data.json` is newline-delimited JSON — one object per line. Images are linked to rows via `frame_index`.

### Row schema

```json
{
  "linear_velocity":      0.35,
  "angular_velocity":    -0.12,
  "gps_latitude":         37.422,
  "gps_longitude":      -122.084,
  "gps_altitude":          10.3,
  "orientation":           45.7,
  "gps_status":               0,
  "gps_service":              1,
  "gps_covariance": [0.01,0,0,0,0.01,0,0,0,0.01],
  "gps_covariance_type":      2,
  "relative_time":        12.34,
  "frame_index":            123,
  "accelerometer_x":     -0.862,
  "accelerometer_y":     -0.362,
  "accelerometer_z":     -0.344,
  "gyroscope_x":          0.370,
  "gyroscope_y":          0.000,
  "gyroscope_z":          0.000,
  "magnetometer_x":     -5262.0,
  "magnetometer_y":      1678.0,
  "magnetometer_z":      3415.0,
  "roll":               -133.68,
  "pitch":                59.95,
  "yaw":                 -77.14
}
```

Sensor fields are `null` when the sensor has not yet provided data.

### Load in Python

```python
import json
from pathlib import Path

session = Path("~/.cache/scout/lab/session_20260602_143022").expanduser()
rows = [json.loads(l) for l in (session / "data.json").read_text().splitlines() if l]

# Link to camera images
def frame_path(session_dir, camera_name, frame_index):
    return session_dir / "image" / camera_name / f"frame_{frame_index:06d}.jpg"
```

---

## Dependencies

```bash
pip install opencv-python-headless pynput requests daily-python onvif-zeep hid piper-tts
sudo apt install ffmpeg
source /opt/ros/humble/setup.bash
```

| Package | Used by | Purpose |
|---------|---------|---------|
| `opencv-python-headless` | cameras, stream, image_writer | Capture, resize, JPEG encode |
| `pynput` | teleop | `r` key listener |
| `requests` | stream | Daily REST API token |
| `daily-python` | stream | Virtual camera + mic |
| `onvif-zeep` | ptz | ONVIF PTZ control |
| `hid` | lights | USB HID relay board |
| `piper-tts` | audio | `PiperVoice` in-process TTS synthesis |
| `rclpy` + `geometry_msgs` | motion, sensors | ROS2 publisher + GPS subscriber |
| `ffmpeg` (binary) | stream | RTSP audio capture for Daily mic |
| `pactl` (binary) | audio | PulseAudio volume + device selection |
