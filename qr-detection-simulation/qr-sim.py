import time
import cv2
from pymavlink import mavutil
from pyzbar.pyzbar import decode as zbar_decode

# --- Connect ---
print("Connecting to SITL...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print(f"Heartbeat received! System: {master.target_system}, Component: {master.target_component}")


def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb is not None and hb.custom_mode == mode_id:
            print(f"Mode confirmed: {mode}")
            break


def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Armed confirmed.")
            break


def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg is not None:
            current_alt = msg.relative_alt / 1000.0
            print(f" Altitude: {current_alt:.2f} m / target {altitude} m")
            if current_alt >= altitude * 0.95:
                print("Target altitude reached.")
                break
        else:
            print(" No GLOBAL_POSITION_INT received in last 3s — waiting...")

def send_velocity(vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )


def wait_until_disarmed():
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb is not None and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Disarm confirmed. Landing complete.")
            break


# --- Vision setup ---
qr_detector = cv2.QRCodeDetector()
cap = cv2.VideoCapture(0)  # your laptop webcam
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam.")

FRAME_CENTER_TOLERANCE = 30   # pixels; how close to center counts as "centered"
GAIN = 0.005                  # converts pixel offset -> m/s velocity (tune this)
MAX_SPEED = 0.5               # m/s cap, keep it gentle in sim


def get_qr_offset():
    """Returns (dx, dy, decoded_text) of QR center from frame center, or (None, None, None) if not found."""
    ret, frame = cap.read()
    if not ret:
        return None, None, None

    h, w = frame.shape[:2]
    frame_cx, frame_cy = w / 2, h / 2

    results = zbar_decode(frame)

    if results:
        qr = results[0]  # take the first detected code if multiple are visible
        decoded_text = qr.data.decode('utf-8')

        # pyzbar gives a bounding rect (left, top, width, height)
        x, y, rw, rh = qr.rect
        qr_cx = x + rw / 2
        qr_cy = y + rh / 2

        dx = qr_cx - frame_cx
        dy = qr_cy - frame_cy

        # Visual feedback
        cv2.rectangle(frame, (x, y), (x + rw, y + rh), (0, 255, 0), 2)
        cv2.putText(frame, decoded_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(frame, (int(qr_cx), int(qr_cy)), 8, (0, 255, 0), -1)
        cv2.circle(frame, (int(frame_cx), int(frame_cy)), 5, (0, 0, 255), -1)
        cv2.imshow("QR Tracking", frame)
        cv2.waitKey(1)

        return dx, dy, decoded_text

    cv2.imshow("QR Tracking", frame)
    cv2.waitKey(1)
    return None, None, None

def precision_land(max_duration=60):
    """Descend while continuously re-centering on the QR code."""
    print("Starting precision landing descent...")
    current_alt = None
    lost_count = 0
    start = time.time()
    descent_speed = 0.3  # m/s downward once centered

    while time.time() - start < max_duration:
        # Non-blocking altitude check — don't stall the vision loop waiting for a message
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg is not None:
            current_alt = msg.relative_alt / 1000.0

        dx, dy, decoded_text = get_qr_offset()

        if dx is None:
            lost_count += 1
            print(f" QR lost during descent ({lost_count}/20), holding position.")
            send_velocity(0, 0, 0)
            if lost_count >= 20:  # ~2 seconds of continuous loss
                print("QR lost too long — aborting descent, falling back to normal LAND.")
                return
        else:
            lost_count = 0
            vy = max(min(dx * GAIN, MAX_SPEED), -MAX_SPEED)
            vx = max(min(-dy * GAIN, MAX_SPEED), -MAX_SPEED)

            # Only descend while reasonably centered; otherwise pause descent and just re-center
            if abs(dx) < FRAME_CENTER_TOLERANCE and abs(dy) < FRAME_CENTER_TOLERANCE:
                vz = descent_speed
            else:
                vz = 0

            print(f" alt={current_alt} | dx={dx:.1f} dy={dy:.1f} | vz={vz}")
            send_velocity(vx, vy, vz)

            if current_alt is not None and current_alt <= 1.0:
                print("Low enough — handing off to LAND mode for touchdown.")
                return

        time.sleep(0.1)

    print("Precision landing timed out — handing off to LAND mode.")

# --- Sequence ---
print("Setting GUIDED mode...")
set_mode("GUIDED")

print("Arming...")
arm()

print("Taking off to 5m...")
takeoff(5)

print("Searching for QR code... hold one up to your webcam.")
centered_count = 0

try:
    while True:
        dx, dy, decoded_text = get_qr_offset()

        if dx is None:
            print(" No QR detected, holding position.")
            send_velocity(0, 0, 0)
        else:
            print(f" QR offset -> dx={dx:.1f}px, dy={dy:.1f}px | Content: {decoded_text}")

            if abs(dx) < FRAME_CENTER_TOLERANCE and abs(dy) < FRAME_CENTER_TOLERANCE:
                centered_count += 1
                send_velocity(0, 0, 0)
                print(f" Centered! ({centered_count}/10) | QR says: {decoded_text}")
                if centered_count >= 10:
                    print(f"Stable center achieved over QR code: '{decoded_text}'")
                    break
            else:
                centered_count = 0
                vy = max(min(dx * GAIN, MAX_SPEED), -MAX_SPEED)
                vx = max(min(-dy * GAIN, MAX_SPEED), -MAX_SPEED)
                send_velocity(vx, vy, 0)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Interrupted by user.")

precision_land()

cap.release()
cv2.destroyAllWindows()

print("Holding briefly before landing...")
for _ in range(20):
    send_velocity(0, 0, 0)
    time.sleep(0.1)

print("Landing...")
set_mode("LAND")
wait_until_disarmed()

print("Mission complete.")
