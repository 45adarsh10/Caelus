# SITL + Gazebo + Webcam Setup (WSL2)

This document covers how to bring up the full simulation stack used by the
scripts in this repo — ArduPilot SITL, classic Gazebo, and (for
vision-based scripts) a passed-through webcam — plus the recurring issues
encountered while getting this working, so they don't need to be
rediscovered from scratch each time.

Environment: Windows 11 + WSL2 (Ubuntu 22.04), classic Gazebo 11.10.2,
ArduPilot (ArduCopter, `gazebo-iris` frame).

## One-time setup

These only need to be done once per machine.

### 1. Install ArduPilot and build SITL for the `gazebo-iris` frame
Follow ArduPilot's own build instructions for your platform. Confirm the
build works with a basic (non-Gazebo) SITL run before adding Gazebo into
the mix — isolates whether a future problem is Gazebo-related or not.

### 2. Install classic Gazebo 11 and the ArduPilot Gazebo plugin
Check your Gazebo version with:
```bash
gazebo --version
```
This repo's setup was built against **classic Gazebo** (`gazebo`), not the
newer `gz sim` (Gazebo Garden/Harmonic) — the two have different launch
commands and are not interchangeable. If `gz sim --version` responds instead
of `gazebo --version`, you're on the newer generation and the commands below
won't directly apply.

### 3. Install `usbipd-win` (Windows side, for webcam access)
WSL2 does not automatically pass through USB devices (including webcams).
On **Windows**, in an Administrator PowerShell:
```powershell
winget install usbipd
```
If `usbipd` isn't recognized immediately after install, open a **new**
terminal window (or restart Windows) — this is a PATH-refresh issue, not a
failed install.

### 4. Bind your webcam (one-time, Windows side, Admin PowerShell)
List devices to find your webcam's BUSID:
```powershell
usbipd list
```
Bind it (only needs to be done once ever, per device):
```powershell
usbipd bind --busid <BUSID>
```

### 5. Add your WSL user to the `video` group (one-time, inside WSL)
```bash
sudo usermod -aG video $USER
```
Close and reopen your WSL terminal afterward for this to take effect. Verify
with:
```bash
groups
```
`video` should be listed.

## Every-session startup sequence

These steps need to be repeated **every time** you start a new session —
after a reboot, sleep/wake cycle, or closing your WSL terminals. Unlike the
one-time steps above, `usbipd attach` and all simulation processes do not
persist.

### Step 1 — Re-attach the webcam (Windows PowerShell, admin not required)
```powershell
usbipd attach --wsl --busid <BUSID>
```
Verify inside WSL:
```bash
ls /dev/video*
```
You should see `/dev/video0` (and often `/dev/video1` — some webcams expose
two device nodes; this is normal).

### Step 2 — Terminal 1: start Gazebo
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```
Run this from the directory containing your `worlds/` folder (commonly
inside the `ardupilot_gazebo` plugin repo, not the main `ardupilot` repo).
**Wait for the 3D window to fully load** before proceeding — starting SITL
before Gazebo is ready can cause the link between them to silently fail.

### Step 3 — Terminal 2: start ArduPilot SITL
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out=127.0.0.1:14550
```
Watch the console for `pre-arm good`. **Important:** this can flicker
between `pre-arm good` and `pre-arm fail` a few times right after startup as
the EKF/GPS finish settling (see Issue 4 below) — wait for it to look
*stable* for a few seconds before running any script that arms the vehicle.

### Step 4 — Terminal 3: run your script
```bash
python3 your_script.py
```

## Troubleshooting log

Issues actually encountered while building this repo, in the order they
tend to bite.

### Issue 1 — `gazebo-iris` frame requires Gazebo running separately
**Symptom:** SITL starts but never receives sensor data; EKF/GPS never
initializes properly, or SITL just hangs.
**Cause:** the `gazebo-iris` frame type expects an already-running Gazebo
instance to connect to. Unlike SITL's default frame (a pure software
physics model), it does not simulate physics on its own.
**Fix:** always start Gazebo first (Step 2 above) and confirm the 3D window
is fully loaded before starting SITL.

### Issue 2 — Webcam fails to open (`can't open camera by index`)
**Symptom:**
```
[ WARN:0] ... VIDEOIO(V4L2:/dev/video0): can't open camera by index
RuntimeError: Could not open webcam.
```
**Cause:** almost always one of two things:
- The USB webcam was never attached to WSL for this session
  (`usbipd attach` is not persistent — see every-session Step 1).
- Your user isn't in the `video` group, or the group change hasn't been
  applied to the current terminal session yet (see one-time Step 5).

**Fix:** re-run `usbipd attach`, confirm `/dev/video0` exists, confirm
`groups` includes `video`. A full reboot or laptop sleep/wake cycle will
reliably re-break the `usbipd attach` part — this is not a one-time fix.

### Issue 3 — Webcam opens but image is mostly green (~80% green, small real image area)
**Symptom:** the OpenCV preview window opens, but only a small portion
shows real video; the rest is solid green.
**Cause:** a resolution/colorspace mismatch between what the camera sends
and what OpenCV's default buffer expects — common with GStreamer-backed
webcam capture on WSL.
**Fix:** explicitly set the FourCC pixel format *and* resolution right
after opening the capture:
```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```
Setting resolution alone was not sufficient in testing — the FourCC line
was the part that actually resolved it.

### Issue 4 — Script hangs forever inside `arm()`
**Symptom:** script prints `Arming...` and never proceeds; no error, no
timeout.
**Cause:** the arm command was sent during a brief window where ArduPilot's
PreArm checks had flickered back to failing (commonly
`Arm: Need Position Estimate`) even though `pre-arm good` had appeared
moments earlier. The GPS/EKF position estimate can take a few extra seconds
to fully stabilize after the first `pre-arm good` message. Since a naive
`arm()` implementation sends the command exactly once and then waits
indefinitely for a confirmation that will never come, this produces a
silent, permanent hang.
**Fix (short-term):** confirm `pre-arm good` has been stable for several
seconds in the SITL console before running a script that arms.
**Fix (robust):** make `arm()` retry with a bounded number of attempts
instead of sending the command once:
```python
def arm(max_attempts=10):
    for attempt in range(max_attempts):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        for _ in range(4):
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Armed confirmed.")
                return
        print(f" Arm attempt {attempt+1} not yet confirmed, retrying...")
    raise RuntimeError("Failed to arm after multiple attempts — check PreArm messages in SITL console.")
```

### Issue 5 — Connected over TCP but never receive `GLOBAL_POSITION_INT` (or any telemetry)
**Symptom:** `wait_heartbeat()` succeeds, but `recv_match()` for any
telemetry message type times out indefinitely, even after requesting a data
stream.
**Cause:** SITL exposes multiple MAVLink endpoints — a raw internal serial
port (e.g. `tcp:127.0.0.1:5762`) and the MAVProxy-managed UDP relay (e.g.
`udp:127.0.0.1:14550`, matching whatever `--out=` was passed to
`sim_vehicle.py`). The raw TCP serial port does not necessarily honor
`request_data_stream_send()` the way the actively-managed UDP link does.
**Fix:** connect over the UDP port that matches your `sim_vehicle.py
--out=` argument, not the raw TCP port:
```python
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
```
Also explicitly request the data stream after the heartbeat, since it is
not guaranteed to be on by default:
```python
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
)
```

### Issue 6 — After a laptop sleep/wake or reboot, everything breaks at once
**Cause:** sleep/reboot drops the USB passthrough (Issue 2), can leave
WSLg's display connection unstable, and kills all running Gazebo/SITL
processes.
**Fix:** don't try to patch pieces individually — close all terminals and
redo the full every-session startup sequence from Step 1. Attempting partial
recovery tends to leave the stack in an inconsistent state that's harder to
debug than a clean restart.

### Issue 7 — QR/barcode decoding unreliable with OpenCV's built-in `QRCodeDetector`
**Symptom:** console repeatedly prints
`Library QUIRC is not linked. No decoding is performed.`
**Cause:** OpenCV's `detectAndDecode()` relies on the QUIRC library for
decoding, which is not linked in this OpenCV build. Detection-only
(`detect()`, geometry without decoding) still works, but actual content
decoding does not.
**Fix:** use `pyzbar` instead, which wraps the `zbar` library and handles
both detection and decoding independently of OpenCV's QUIRC dependency:
```bash
sudo apt install -y libzbar0
pip3 install pyzbar
```
```python
from pyzbar.pyzbar import decode as zbar_decode
results = zbar_decode(frame)
```

## Useful diagnostic commands

Check for duplicate/stale processes (can cause jittery physics or port
conflicts if old sessions weren't fully closed):
```bash
ps aux | grep -E "gzserver|gzclient|arducopter|mavproxy|sim_vehicle" | grep -v grep
```

Kill everything and start clean:
```bash
pkill -9 gzserver
pkill -9 gzclient
pkill -9 arducopter
pkill -9 -f mavproxy.py
pkill -9 -f sim_vehicle.py
```

Check what's listening on a given MAVLink port:
```bash
sudo lsof -i :5762
```
