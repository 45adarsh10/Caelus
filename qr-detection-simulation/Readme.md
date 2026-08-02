# QR Code Detection & Precision Landing (Simulation)

A simulated ArduCopter drone (ArduPilot SITL + Gazebo) autonomously detects a
QR code via a live webcam feed, centers itself above it using proportional
velocity control, and performs a precision landing — continuously
re-centering on the code throughout the descent rather than descending blind.



## What it does, step by step

1. Connects to ArduPilot SITL over MAVLink (UDP).
2. Arms and takes off to 5m altitude in GUIDED mode.
3. Continuously scans webcam frames for a QR code using `pyzbar`.
4. Once found, computes the pixel offset between the QR code's center and
   the frame's center, and sends proportional velocity commands to close
   that gap.
5. Once centered and stable for ~1 second, begins descending — but keeps
   detecting and re-centering on every frame throughout the descent (true
   precision landing, not "center once then land blind").
6. Hands off to ArduPilot's own `LAND` mode for the final meter of descent
   and touchdown.
7. Disarms and exits cleanly.

## Why a webcam instead of a simulated drone-mounted camera?

Gazebo can attach a virtual camera directly to the simulated drone, but that
requires simulator-specific plumbing (Gazebo transport topics) that has no
equivalent on real hardware. A real drone's camera is just a physical camera
plugged into a companion computer, read with plain `cv2.VideoCapture()`.
Using a laptop webcam here means the entire vision/control pipeline in this
script — the actually valuable part — transfers directly to a Raspberry Pi
or Jetson later, with no rewrite needed.

## Setup

**Environment:** Ubuntu 22.04 (tested in WSL2), ArduPilot SITL, classic
Gazebo 11.

**System dependency** (not covered by `requirements.txt`, since it's not a
Python package):
```bash
sudo apt install -y libzbar0
```

**Python dependencies:**
```bash
pip3 install -r requirements.txt
```

**Simulation stack** (Gazebo + SITL) must be running before this script —
see `../docs/setup-wsl-sitl-gazebo.md` for the full startup sequence and
common troubleshooting (this took a fair amount of debugging to get right:
USB webcam passthrough on WSL, EKF arming race conditions, TCP vs UDP
telemetry stream issues, and OpenCV/webcam colorspace bugs are all covered
there).

## Run

With Gazebo and SITL already running:
```bash
python3 day12.py
```
Hold a QR code up to your webcam once the console prints
"Searching for QR code...".

## Known limitations

- Detection reliability depends on lighting, angle, distance, and QR code
  size in frame — `pyzbar` occasionally drops detection for a frame or two
  even with the code held steady; the script tolerates brief loss (up to
  ~2 seconds) without aborting.
- `GAIN` and `descent_speed` are tuned loosely for this simulation and would
  need retuning for real hardware.
- Centering tolerance (`FRAME_CENTER_TOLERANCE = 30px`) and the final
  handoff altitude (1.0m) are simulation-appropriate defaults, not
  validated for real-world precision landing yet.
