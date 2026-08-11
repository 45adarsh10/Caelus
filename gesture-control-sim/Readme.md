# Gesture-Controlled Drone

Fly an ArduCopter-based drone with hand gestures, using a webcam, [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker) hand tracking, and [MAVLink](https://mavlink.io/en/) (via `pymavlink`). Built for a Pixhawk/Cube-family flight controller running ArduCopter, and tested against both real hardware and [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).

> **Safety notice:** this is student/hobbyist project code, not certified flight software. Test in [SITL](#testing-in-simulation-sitl--gazebo) first, then on a bench with props off, then in open space with no obstacles or people nearby, and always keep an RC transmitter ready to override into another flight mode. See [Safety notes](#safety-notes) below for what this script does and does not protect against.

## Gestures

All gestures except `THUMBS_UP` only act while the vehicle is in the matching flight state (see [State machine](#state-machine) below).

| Gesture | Hand shape | State | Action |
|---|---|---|---|
| `THUMBS_UP` | Fist, thumb out, pointed up | On ground | Arm + takeoff |
| `FIST` | All fingers curled, thumb tucked | Flying | Continuously descend; auto-lands once altitude reaches `MIN_DESCEND_ALTITUDE` |
| `OPEN_PALM` | 3+ fingers extended | Flying | Hold position and altitude (stays in `GUIDED`) |
| `ROCK_ON` | Index + pinky extended | Flying | Ascend continuously while held, capped at `MAX_ALTITUDE` |
| `THUMBS_RIGHT` / `THUMBS_LEFT` | Fist, thumb out, pointed sideways | Flying | Yaw right / left |
| `PEACE_SIGN` | Index + middle extended | Flying | Land immediately, from current altitude (skips `FIST`'s gradual descent) |
| `POINT_UP` | Index only, pointed up | Flying | Move forward while held |
| `POINT_DOWN` | Index only, pointed down | Flying | Move backward while held |
| `POINT_LEFT` | Index only, pointed left | Flying | Strafe left while held |
| `POINT_RIGHT` | Index only, pointed right | Flying | Strafe right while held |

All "while held" gestures are continuous inputs — release the gesture to stop that motion. Movement/yaw commands are sent as body-frame velocity setpoints, so "forward" always means the direction the vehicle's nose is currently facing, not a compass direction.

### State machine

```
ON_GROUND -> TAKING_OFF -> FLYING -> LANDING -> ON_GROUND
```

State transitions are driven by real telemetry (altitude, armed status), not by assuming a command succeeded.

## Installation

```bash
pip install -r requirements.txt
```

You'll also need MediaPipe's hand landmark model file. Download `hand_landmarker.task` from the [MediaPipe model zoo](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker) and place it in the repo root (or point `--model` at wherever you put it).

## Usage

```bash
python gesture_drone.py [--connect CONNECTION_STRING] [--model MODEL_PATH] [--camera-index N]
```

| Flag | Default | Notes |
|---|---|---|
| `--connect` | `udpin:0.0.0.0:14550` | MAVLink connection string. Matches ArduPilot SITL's default MAVProxy output port — no override needed for SITL. |
| `--model` | `hand_landmarker.task` | Path to the MediaPipe model file. |
| `--camera-index` | `0` | OpenCV camera index, if you have more than one webcam. |

Press `q` in the video window to quit. If the vehicle is airborne when the script exits (crash, camera loss, or quit), it sends an emergency `LAND` before shutting down.

## Testing in simulation (SITL + Gazebo)

Recommended before touching real hardware. This gives you a full 3D view of the vehicle so you can visually confirm each movement gesture actually moves it in the expected direction.

1. **Install ArduPilot SITL** (if you don't already have it):
   ```bash
   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   . ~/.profile
   ```
2. **Install Gazebo (Harmonic) and the ArduPilot plugin**: follow the [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) README.
3. **Launch Gazebo** with a copter world:
   ```bash
   gz sim -v4 -r iris_runway.sdf
   ```
4. **Launch SITL** pointed at Gazebo, in a second terminal (from `ardupilot/ArduCopter`):
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
   ```
5. **Wait for GPS/EKF ready** — watch the SITL console for a line like `EKF3 IMU0 is using GPS` before arming; pre-arm checks will otherwise reject `THUMBS_UP`.
6. **Run this script** with no `--connect` override needed (its default matches SITL's default output port), and gesture at your real webcam while watching the Gazebo viewport respond.

If you don't need the 3D visual and just want a quick check, plain SITL without Gazebo (`sim_vehicle.py -v ArduCopter --console --map`) works too and is much lighter on CPU/GPU.

## Configuration

Key tunables live as constants near the top of `gesture_drone.py` (not CLI flags, since they're flight-behavior tuning rather than per-run options):

| Constant | Default | Meaning |
|---|---|---|
| `TARGET_TAKEOFF_ALT` | 5.0 m | Altitude `THUMBS_UP` takeoff climbs to |
| `MAX_ALTITUDE` | 15.0 m | Ceiling `ROCK_ON` ascend won't exceed |
| `MIN_DESCEND_ALTITUDE` | 1.0 m | Floor `FIST` descends to before triggering real `LAND` |
| `CLIMB_RATE` / `DESCEND_RATE` | 0.7 / 0.5 m/s | Vertical speed for `ROCK_ON` / `FIST` |
| `MOVE_SPEED` | 0.5 m/s | Horizontal speed for `POINT_UP/DOWN/LEFT/RIGHT` |
| `YAW_RATE_DPS` | 45 deg/s | Yaw rate for `THUMBS_LEFT/RIGHT` |
| `GESTURE_WINDOW_SIZE` / `GESTURE_MIN_VOTES` | 7 / 5 | Rolling-window majority vote for gesture confirmation — higher `GESTURE_MIN_VOTES` is stricter but slower to register |
| `THUMB_EXTENSION_MARGIN` / `FINGER_EXTENSION_MARGIN` | 1.15 / 1.2 | Lower = more permissive finger-extended detection |

If a gesture isn't registering reliably in your lighting/camera setup, watch the on-screen `T:I:M:R:P` finger-state readout while holding the gesture and nudge the extension margins or angle thresholds.

## Safety notes

- Sends periodic heartbeats so ArduCopter's GCS failsafe doesn't trigger on its own.
- Listens to `COMMAND_ACK` / `STATUSTEXT` so a rejected `ARM`/`TAKEOFF` isn't silent.
- Flags unexpected altitude loss while `FLYING` as a likely vehicle-side failsafe, distinct from an intentional `FIST` descent.
- Sends an emergency `LAND` on any unexpected script exit while airborne.
- Flags a stale MAVLink link (no messages for `LINK_TIMEOUT` seconds).
- **Horizontal movement gestures have no obstacle or geofence awareness.** `POINT_UP/DOWN/LEFT/RIGHT` will fly the vehicle into whatever's in front of it — this script has no positional or collision sensing. Only fly these in open space you've checked yourself, or in simulation.
- **`FIST` has no time cap on its descent.** An unreleased fist will ride the vehicle all the way down to `MIN_DESCEND_ALTITUDE` and auto-land. Use `PEACE_SIGN` if you want an instant landing instead of the gradual descent.

See the module docstring in `gesture_drone.py` for a full changelog of design decisions and fixes (v1–v8).

## License

No license file is included yet — add one (e.g. MIT, Apache-2.0) appropriate for your team/competition before making this repo public, if you haven't already settled on one.
