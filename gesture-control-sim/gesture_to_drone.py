"""
Gesture-controlled drone via webcam + MediaPipe hand tracking + MAVLink (ArduCopter).

Gestures (only act while the drone is in the matching state):
    THUMBS_UP                 (on ground)    -> arm + takeoff
    FIST                      (while flying) -> continuously descend; once altitude
                                                 reaches MIN_DESCEND_ALTITUDE, LAND
                                                 fires automatically (see handle_fist())
    OPEN_PALM                 (while flying) -> hold position/altitude (stays in GUIDED)
    ROCK_ON                   (while flying) -> ascend continuously while held, capped at
                                                 MAX_ALTITUDE (index + pinky extended, middle
                                                 + ring curled)
    THUMBS_RIGHT / THUMBS_LEFT (while flying) -> yaw right / left
      (a fist with the thumb out, pointed sideways instead of up)
    PEACE_SIGN                (while flying) -> land immediately, from whatever
                                                 altitude you're at right now
                                                 (index + middle extended; a discrete,
                                                 cooldown-gated override -- distinct from
                                                 FIST's gradual descend-to-floor-then-land)
    POINT_UP / POINT_DOWN / POINT_LEFT / POINT_RIGHT (while flying) -> move forward / backward /
                                                 strafe left / strafe right while held (index
                                                 finger only, direction read from which way it
                                                 points; POINT_UP means "move forward", not
                                                 "ascend" -- ROCK_ON above handles ascend)

Safety notes:
    - Sends periodic heartbeats so ArduCopter's GCS failsafe doesn't trigger.
    - Listens to COMMAND_ACK / STATUSTEXT so a failed ARM/TAKEOFF isn't silent.
    - State machine is driven by real telemetry (altitude), not assumptions
      about whether a command succeeded.
    - Flags unexpected altitude loss while "FLYING" as a likely vehicle-side
      failsafe rather than silently mislabeling it.
    - Sends an emergency LAND on any unexpected exit while airborne.
    - Flags a stale MAVLink link (no messages for LINK_TIMEOUT seconds) instead
      of silently continuing to send commands into a dead connection.

v3 fix: OPEN_PALM used to switch the vehicle into ALT_HOLD. ALT_HOLD and
LOITER are manual, RC-stick-driven modes -- they expect a live throttle
signal, which this script never sends (no RC_CHANNELS_OVERRIDE). The moment
the vehicle entered ALT_HOLD with zero throttle input, ArduCopter's landing
detector decided it was on the ground and auto-disarmed, which looked like
"open palm lands the drone." The fix: never leave GUIDED. Holding position
is done by continuously streaming a zero-velocity SET_POSITION_TARGET_LOCAL_NED
setpoint every loop tick while flying (a GUIDED setpoint is a stream, not a
one-shot command -- sending it once isn't enough to hold).

v4 fix (gesture recognition hardening): two changes address gestures that
were being missed or flickering in practice:

  1. Finger "extended" was decided from joint angle alone. Angle degrades
     gracefully near its threshold (e.g. a finger held rigid but not fully
     unfurled can still read >150deg), which let borderline frames flip a
     finger's state and, in turn, flip the whole gesture (a real problem
     for the thumb specifically -- a fist with the thumb tucked over the
     folded fingers can still show a fairly open CMC-MCP-TIP angle, since
     the thumb has one fewer joint to fold, misreading a FIST as a
     THUMBS_* gesture). Every finger now also needs a *distance* check:
     the fingertip has to be meaningfully farther from the wrist than that
     finger's own base knuckle is, normalized by a hand-scale reference
     (wrist-to-middle-MCP distance) so it works at any distance from the
     camera. A finger only counts as extended when both the angle and the
     distance checks agree, which is what actually separates "genuinely
     unfolded" from "borderline."
  2. Confirmation used a consecutive-frame streak: one misclassified frame
     in the middle of an otherwise-clean, held gesture reset the whole
     streak to zero, which could make a real gesture take much longer to
     register than GESTURE_WINDOW_SIZE frames, or never register at all
     under mild jitter. Replaced with a rolling-window majority vote:
     confirm the gesture that has GESTURE_MIN_VOTES or more of the last
     GESTURE_WINDOW_SIZE classified frames, which tolerates the occasional
     bad frame without needing a perfect run.

  The on-screen T/I/M/R/P readout still reflects live finger_states, so if
  a gesture still isn't registering reliably in your lighting/camera setup,
  watch that readout while holding the gesture and nudge
  THUMB_EXTENSION_MARGIN / FINGER_EXTENSION_MARGIN (lower = more permissive)
  or the angle thresholds below.

v5 fix (flight envelope + robustness):
  1. POINT_UP had no ceiling -- continuous_ascend() now refuses to extend
     the climb window once altitude >= MAX_ALTITUDE.
  2. There was no controlled descent, only instant LAND. FIST now performs
     a bounded continuous descent for as long as it's held (floored, at the
     time, at MIN_DESCEND_ALTITUDE) so it never tried to land itself via
     gesture control alone.
  3. All MAVLink sends now go through _safe_send(), which logs failures
     instead of raising into the main loop, and a link-health check flags
     when no MAVLink message has arrived for LINK_TIMEOUT seconds.
  4. MAVLINK_CONN / MODEL_PATH are now just defaults -- overridable via
     --connect / --model / --camera-index.
  5. The MediaPipe model file's existence is checked up front with a clear
     error instead of letting MediaPipe raise an opaque one.

v6 fix (altitude-triggered FIST landing): FIST used to trigger the real LAND
command purely on a hold *duration* (FIST_LAND_HOLD_DURATION) -- so an
accidental 1.5s-plus fist hold at any altitude, including well above
MAX_ALTITUDE, would fire a full autonomous landing from height. handle_fist()
is now driven by altitude instead of a timer: it continuously descends the
vehicle for as long as FIST is held, and LAND fires only once altitude
actually reaches MIN_DESCEND_ALTITUDE. This bounds "how high can a fist
misfire drop you from" by altitude rather than by how long the hand
happened to stay in a fist shape. The tradeoff: an unreleased fist (gesture
misfire, distracted user) will now ride the vehicle all the way down and
auto-land, with no time-based cap on that -- there's no longer a way to
"briefly" fist without continuing to lose altitude, so release the gesture
promptly once you're at the height you want. FIST_LAND_HOLD_DURATION is
no longer used and has been removed.

v7 additions (instant land + horizontal movement):
  1. _update_state_from_telemetry()'s "altitude dropping unexpectedly"
     failsafe warning used to fire on ANY altitude drop below 50% of
     TARGET_TAKEOFF_ALT while state==FLYING with no LAND sent -- including
     a perfectly normal, intentional FIST descent in progress, since that
     also drops altitude below the threshold before LAND actually fires at
     MIN_DESCEND_ALTITUDE. It's now gated on time.time() >=
     self._descend_until, so it only fires when there is NOT an active
     FIST-driven descend window -- an intentional gesture descent no
     longer gets misreported as a vehicle-side failsafe.
  2. Added PEACE_SIGN (index + middle extended) as a discrete, immediate
     LAND override -- skips FIST's gradual descend-to-floor ramp and lands
     from whatever altitude the vehicle is at right now.
  3. Added horizontal movement: ROCK_ON (index + pinky extended) moves
     forward while held; POINT_DOWN/POINT_LEFT/POINT_RIGHT (the same
     index-only shape as POINT_UP, just pointed in a different direction)
     strafe backward/left/right while held. These stream vx/vy through the
     same GUIDED velocity setpoint as climb/descend/yaw (see move() and
     _stream_hold_if_flying()). Unlike vertical movement, there is no
     altitude/geofence-style cap on horizontal movement -- this script has
     no awareness of what's actually around the vehicle, so movement
     gestures are only as safe as the physical space you fly them in.

v8 change (gesture reassignment): POINT_UP now means "move forward"
instead of "ascend" -- ascend moved to ROCK_ON (index + pinky extended),
which previously meant "move forward". This is a pure reassignment of
which action each gesture triggers; no new hand shapes were added and no
existing shape's classification logic changed. Note the v5 fix note above
(POINT_UP had no ceiling...) is describing history accurately as written
at the time -- MAX_ALTITUDE now caps ROCK_ON's climb, not POINT_UP's.
"""

import argparse
import os
import time
import math
from collections import namedtuple, deque, Counter

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from pymavlink import mavutil


# ======================= Configuration =======================

MODEL_PATH = 'hand_landmarker.task'      # default; override with --model
MAVLINK_CONN = 'udpin:0.0.0.0:14550'     # default; override with --connect

TARGET_TAKEOFF_ALT = 5.0        # meters
TAKEOFF_ARM_DELAY = 1.0         # seconds between ARM and TAKEOFF commands
COMMAND_COOLDOWN = 3.0          # seconds between repeats of the same discrete command
HEARTBEAT_INTERVAL = 1.0        # seconds; needed to avoid ArduCopter GCS failsafe
HOLD_STREAM_INTERVAL = 0.25     # seconds; rate to re-send the GUIDED hold setpoint (4 Hz)
LINK_TIMEOUT = 5.0              # seconds with no MAVLink message of any kind before we flag the link as lost

CLIMB_RATE = 0.7                # m/s, vertical speed while ROCK_ON is being held
CLIMB_HOLD_GRACE = 0.4          # seconds; each ROCK_ON/FIST frame pushes the climb/descend
                                 # window forward by this much, so the motion keeps going for
                                 # as long as the gesture is held rather than firing a
                                 # fixed-length burst. Must exceed HOLD_STREAM_INTERVAL, with
                                 # margin for frame-rate jitter, or it will stutter (drop to 0
                                 # for a tick) between frames.
MAX_ALTITUDE = 15.0             # meters; continuous_ascend() will not extend the climb window
                                 # once at/above this, capping how high ROCK_ON can take the vehicle

DESCEND_RATE = 0.5              # m/s, vertical speed while FIST is held and above MIN_DESCEND_ALTITUDE
MIN_DESCEND_ALTITUDE = 1.0      # meters; FIST descends the vehicle down to this floor, at which
                                 # point handle_fist() fires the real LAND command and lets
                                 # ArduCopter's own landing controller take it the rest of the way

YAW_RATE_DPS = 45.0             # deg/s while a THUMBS_RIGHT/LEFT yaw is active
YAW_DURATION = 1.0              # seconds; how long each yaw burst lasts (~45deg/press)

MOVE_SPEED = 0.5                 # m/s, horizontal speed for POINT_UP/DOWN/LEFT/RIGHT
MOVE_HOLD_GRACE = 0.4            # seconds; each movement-gesture frame pushes the movement
                                  # window forward by this much -- same windowed-push pattern
                                  # as CLIMB_HOLD_GRACE, so movement continues for as long as
                                  # the gesture is held and glides to a stop within this long
                                  # of release. Must exceed HOLD_STREAM_INTERVAL, with margin
                                  # for frame-rate jitter, or movement will stutter between frames.

# How far off-axis a pointing index finger can be before a directional point
# is treated as ambiguous (NONE) rather than forced into a specific movement
# direction. Same role as THUMB_DOMINANCE_RATIO, applied to the index finger
# instead of the thumb.
POINT_DOMINANCE_RATIO = 1.3

STRAIGHT_ANGLE_THRESHOLD = 150       # degrees; general finger counts "extended" above this
THUMB_STRAIGHT_ANGLE_THRESHOLD = 130 # degrees; the thumb's CMC/MCP joint bends more than the
                                      # other fingers even when genuinely extended sideways, so
                                      # it needs a lower bar or a sideways thumb misreads as "not
                                      # extended" and falls out of the THUMBS_* branch entirely

# Distance-based cross-check (see v4 fix note above). A finger/thumb is only
# "extended" if, in addition to the angle test, its tip sits at least this
# many times farther from the wrist than its own base knuckle does. Both
# distances are normalized by hand scale first, so this works regardless of
# how close the hand is to the camera. Lower these (toward 1.0) if a real
# extended finger is being missed; raise them if a curled finger still
# registers as extended (e.g. FIST briefly reading as a THUMBS_* gesture).
THUMB_EXTENSION_MARGIN = 1.15
FINGER_EXTENSION_MARGIN = 1.2

# How far off-axis the thumb can point before a thumb-out gesture is treated
# as ambiguous (NONE) rather than forced into UP/LEFT/RIGHT. Higher = stricter.
THUMB_DOMINANCE_RATIO = 1.3

GESTURE_WINDOW_SIZE = 7         # frames considered for the rolling majority vote
GESTURE_MIN_VOTES = 5           # of GESTURE_WINDOW_SIZE frames that must agree to confirm
SMOOTHING_ALPHA = 0.4           # 0-1, weight on each new frame in the landmark EMA filter;
                                 # lower = smoother/slower to react, higher = noisier/snappier

# ArduCopter custom_mode numbers used here. We deliberately never command
# ALT_HOLD or LOITER -- see the v3 fix note above.
MODE_GUIDED = 4
MODE_LAND = 9

# MediaPipe hand landmark indices
WRIST = 0
THUMB_CMC, THUMB_MCP, THUMB_TIP = 1, 2, 4
FINGER_JOINTS = {  # name: (mcp, pip, tip)
    'index':  (5, 6, 8),
    'middle': (9, 10, 12),
    'ring':   (13, 14, 16),
    'pinky':  (17, 18, 20),
}


# ======================= Gesture recognition =======================

Point3 = namedtuple("Point3", ["x", "y", "z"])


class GestureRecognizer:
    """Classifies a single hand's landmarks into a discrete gesture using
    3D joint angles (robust to hand rotation/tilt toward the camera, unlike
    a raw distance-from-wrist check) cross-checked against normalized
    tip-vs-knuckle distance (robust to the angle metric's own blind spots,
    see the v4 fix note above), plus temporal hysteresis: a gesture only
    counts as "confirmed" once it wins a majority vote over the last
    GESTURE_WINDOW_SIZE frames, so a stray misread frame can't block or
    falsely trigger a command."""

    def __init__(self):
        self._history = deque(maxlen=GESTURE_WINDOW_SIZE)
        self._smoothed_lm = None  # list[Point3]; EMA-smoothed landmark positions

    def _smooth(self, lm):
        """Exponential moving average over landmark positions. Raw per-frame
        landmark noise was flickering the classified gesture (e.g.
        THUMBS_RIGHT <-> NONE) frame to frame. Smoothing damps that jitter
        while SMOOTHING_ALPHA keeps it responsive enough not to noticeably
        lag a real, held gesture change."""
        if self._smoothed_lm is None:
            self._smoothed_lm = [Point3(p.x, p.y, p.z) for p in lm]
        else:
            a = SMOOTHING_ALPHA
            self._smoothed_lm = [
                Point3(a * p.x + (1 - a) * s.x, a * p.y + (1 - a) * s.y, a * p.z + (1 - a) * s.z)
                for p, s in zip(lm, self._smoothed_lm)
            ]
        return self._smoothed_lm

    @staticmethod
    def _angle(a, b, c):
        """3D angle at point b between rays b->a and b->c, in degrees.
        A straight finger reads close to 180 degrees; a curled one much
        less. Using z (MediaPipe's estimated depth) as well as x/y avoids
        misreading a genuinely straight finger as curled just because the
        hand is tilted toward/away from the camera."""
        v1 = (a.x - b.x, a.y - b.y, a.z - b.z)
        v2 = (c.x - b.x, c.y - b.y, c.z - b.z)
        mag1 = math.sqrt(sum(k * k for k in v1))
        mag2 = math.sqrt(sum(k * k for k in v2))
        if mag1 * mag2 == 0:
            return 180.0
        cos_angle = sum(v1[i] * v2[i] for i in range(3)) / (mag1 * mag2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        return math.degrees(math.acos(cos_angle))

    @staticmethod
    def _distance(a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def _hand_scale(self, lm):
        """A hand-size reference (wrist to middle-finger MCP) used to
        normalize the distance checks below so they work the same whether
        the hand fills the frame or is far from the camera."""
        scale = self._distance(lm[WRIST], lm[FINGER_JOINTS['middle'][0]])
        return scale if scale > 1e-6 else 1e-6

    def finger_states(self, lm):
        """Returns {finger_name: extended_bool}. A finger counts as
        extended only when BOTH hold: its joint angle is above threshold,
        AND its tip sits meaningfully farther from the wrist than its own
        base knuckle does (normalized by hand scale). The angle test alone
        can read a curled thumb as borderline-open (it has one fewer joint
        than the other fingers); the distance test catches that case."""
        scale = self._hand_scale(lm)
        states = {}

        thumb_angle = self._angle(lm[THUMB_CMC], lm[THUMB_MCP], lm[THUMB_TIP])
        thumb_tip_dist = self._distance(lm[THUMB_TIP], lm[WRIST]) / scale
        thumb_base_dist = self._distance(lm[THUMB_MCP], lm[WRIST]) / scale
        states['thumb'] = (
            thumb_angle > THUMB_STRAIGHT_ANGLE_THRESHOLD
            and thumb_tip_dist > thumb_base_dist * THUMB_EXTENSION_MARGIN
        )

        for name, (mcp, pip, tip) in FINGER_JOINTS.items():
            angle = self._angle(lm[mcp], lm[pip], lm[tip])
            tip_dist = self._distance(lm[tip], lm[WRIST]) / scale
            base_dist = self._distance(lm[mcp], lm[WRIST]) / scale
            states[name] = (
                angle > STRAIGHT_ANGLE_THRESHOLD
                and tip_dist > base_dist * FINGER_EXTENSION_MARGIN
            )
        return states

    def classify(self, lm):
        """Single-frame gesture classification (no hysteresis applied yet).
        Applies EMA smoothing (see _smooth) first. Returns (gesture_name,
        finger_states_dict). finger_states_dict reflects the smoothed
        landmarks, which is also what the on-screen T/I/M/R/P readout ends
        up showing."""
        lm = self._smooth(lm)
        f = self.finger_states(lm)
        four = [f['index'], f['middle'], f['ring'], f['pinky']]
        extended_count = sum(four)

        if f['thumb'] and extended_count == 0:
            # Classify a fist-with-thumb-out by which axis the thumb
            # points along, not just "is it above the wrist". Requiring
            # one axis to dominate the other by THUMB_DOMINANCE_RATIO keeps
            # a diagonal thumb from flickering between UP/LEFT/RIGHT --
            # combined with the majority-vote confirmation, the gesture has
            # to be a clear, held direction to register at all.
            dx = lm[THUMB_TIP].x - lm[WRIST].x
            dy = lm[THUMB_TIP].y - lm[WRIST].y
            if abs(dy) > abs(dx) * THUMB_DOMINANCE_RATIO and dy < 0:
                return "THUMBS_UP", f
            if abs(dx) > abs(dy) * THUMB_DOMINANCE_RATIO:
                # Landmarks come from the already-mirrored (cv2.flip) frame,
                # same as the wrist-x used for the old swipe detector, so
                # +x here is the user's real right.
                return ("THUMBS_RIGHT" if dx > 0 else "THUMBS_LEFT"), f
            return "NONE", f

        # Require at least 3 of the 4 fingers to read as extended for
        # OPEN_PALM, and check it BEFORE the FIST check. This matters:
        # a slightly bent finger or one noisy landmark can flip a single
        # finger's state below threshold on a real open-hand frame. With
        # a "not any(four)" FIST check evaluated first, that one noisy
        # finger would be enough to fall through and get misread as a
        # fist. Checking OPEN_PALM first with a tolerant 3-of-4 majority
        # makes a genuine open palm win even when one finger is noisy.
        if extended_count >= 3:
            return "OPEN_PALM", f

        if not f['thumb'] and extended_count == 0:
            return "FIST", f

        # PEACE_SIGN (index + middle extended, ring + pinky curled) is
        # checked before POINT_UP so a real 2-finger gesture can't fall
        # through to POINT_UP's 1-finger check on a frame where the middle
        # finger reads borderline. It's used as an instant-LAND override --
        # deliberately a different finger shape than FIST/POINT_UP/OPEN_PALM
        # so it can't be reached by a jittery misread of any of those.
        if f['index'] and f['middle'] and not f['ring'] and not f['pinky'] and extended_count == 2:
            return "PEACE_SIGN", f

        # ROCK_ON (index + pinky extended, middle + ring curled) drives
        # continuous ascend. It's a distinct 2-finger shape from PEACE_SIGN
        # (different pair of fingers), so the two can't be confused with
        # each other even under jitter. It's a dedicated shape rather than
        # a directional point (unlike POINT_UP/DOWN/LEFT/RIGHT below)
        # because vertical climb needs the MAX_ALTITUDE cap in
        # continuous_ascend() to behave differently from horizontal
        # movement, which has no such cap -- keeping it a separate gesture
        # keeps that distinction obvious in the gesture vocabulary too.
        if f['index'] and f['pinky'] and not f['middle'] and not f['ring'] and extended_count == 2:
            return "ROCK_ON", f

        if f['index'] and extended_count == 1:
            # Directional pointing -- same axis-dominance pattern as the
            # thumb branch above (check dy dominance first, then dx),
            # applied to the index fingertip instead of the thumb tip. All
            # four directions (POINT_UP/DOWN/LEFT/RIGHT) drive horizontal
            # movement via move() -- POINT_UP means "move forward", not
            # "ascend" (that's ROCK_ON's job; see the comment above).
            # THUMBS_LEFT/RIGHT (rotate in place) and POINT_LEFT/RIGHT
            # (strafe sideways) use different finger shapes (thumb-out-of-
            # fist vs. index-only) so they can't be confused with each other.
            tip = lm[FINGER_JOINTS['index'][2]]
            dx = tip.x - lm[WRIST].x
            dy = tip.y - lm[WRIST].y
            if abs(dy) > abs(dx) * POINT_DOMINANCE_RATIO:
                return ("POINT_UP" if dy < 0 else "POINT_DOWN"), f
            if abs(dx) > abs(dy) * POINT_DOMINANCE_RATIO:
                return ("POINT_RIGHT" if dx > 0 else "POINT_LEFT"), f
            return "NONE", f

        return "NONE", f

    def confirm(self, raw_gesture):
        """Rolling-window majority vote (see v4 fix note above). Appends
        this frame's raw classification and confirms whichever gesture
        holds at least GESTURE_MIN_VOTES of the last GESTURE_WINDOW_SIZE
        frames -- tolerating an occasional stray misclassification without
        needing a perfect consecutive run."""
        self._history.append(raw_gesture)
        if len(self._history) < GESTURE_WINDOW_SIZE:
            return "NONE"
        gesture, votes = Counter(self._history).most_common(1)[0]
        if gesture != "NONE" and votes >= GESTURE_MIN_VOTES:
            return gesture
        return "NONE"

    def reset(self):
        """Call when no hand is visible so a reappearing hand has to
        re-earn gesture confirmation from scratch. Also clears the
        smoothing buffer -- otherwise a hand reappearing in a completely
        different position would get EMA-blended against its old, now
        stale, smoothed position for the first several frames."""
        self._history.clear()
        self._smoothed_lm = None


# ======================= Drone control =======================

class DroneController:
    """Wraps the MAVLink connection: telemetry polling, command sending,
    command-result / failsafe diagnostics, and a state machine driven by
    real telemetry rather than assumptions about command success."""

    ACK_RESULT_NAMES = {
        0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
        3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS",
    }

    # ArduCopter custom_mode -> name, for the modes this script can encounter
    # (either ones we command, or common failsafe destinations). We keep
    # ALT_HOLD/LOITER in this table purely for diagnostics -- if the vehicle
    # ever ends up in one of these on its own, the ">>> VEHICLE CHANGED MODE
    # ON ITS OWN" print below will name it.
    MODE_NAMES = {
        0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
        6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD", 17: "BRAKE",
        20: "GUIDED_NOGPS", 21: "SMART_RTL", 27: "AUTO_RTL",
    }

    def __init__(self, connection_str):
        print(f"Connecting to drone on '{connection_str}'...")
        self.conn = mavutil.mavlink_connection(connection_str)
        hb = self.conn.wait_heartbeat(timeout=30)
        if hb is None:
            raise RuntimeError(
                f"No heartbeat received on '{connection_str}' within 30s -- is ArduPilot "
                f"SITL (or the real autopilot) running and pointed at this connection "
                f"string? (override with --connect)"
            )
        print(f"Connected to system {self.conn.target_system}")

        self.ack_cmd_names = {
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "ARM/DISARM",
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
            mavutil.mavlink.MAV_CMD_NAV_LAND: "LAND",
            mavutil.mavlink.MAV_CMD_CONDITION_YAW: "YAW",
        }

        # ON_GROUND -> TAKING_OFF -> FLYING -> LANDING -> ON_GROUND
        self.state = "ON_GROUND"
        self.altitude = 0.0
        self.armed = False
        self.heading = None           # compass heading in degrees (0-360), from VFR_HUD
        self.current_mode = None      # last flight mode reported by the vehicle
        self._expected_mode = None    # mode number we most recently commanded
        self.link_lost = False        # True once LINK_TIMEOUT passes with no MAVLink message at all

        self._pending_takeoff_time = None
        self._last_command_time = {}
        self._last_heartbeat_time = 0.0
        self._last_hold_stream_time = 0.0
        self._last_message_time = time.time()

        self._climb_until = 0.0    # time.time() timestamp; while now < this, stream a climb setpoint
        self._descend_until = 0.0  # time.time() timestamp; while now < this (and not climbing), stream a descend setpoint
        self._was_climbing = False    # tracks climb/descend on/off transitions, purely for the log lines below
        self._was_descending = False
        self._fist_confirmed_since = None  # time.time() timestamp FIST started being continuously
                                            # confirmed while FLYING, or None. Display-only now (see
                                            # fist_hold_progress()) -- handle_fist()'s LAND trigger is
                                            # altitude-based, not duration-based.

        self._yaw_until = 0.0     # time.time() timestamp; while now < this, stream the yaw rate below alongside the hold/climb/descend setpoint
        self._active_yaw_rate = 0.0  # rad/s, signed (positive = clockwise/right)

        # Horizontal movement -- same windowed-push pattern as
        # _climb_until/_descend_until above, just on the x (forward/back)
        # and y (right/left) body axes instead of z. Direction is signed
        # (+1/-1) and stored separately from the window timestamp so the
        # stream in _stream_hold_if_flying() knows both whether to move
        # and which way, right up until the window itself expires.
        self._move_x_until = 0.0   # time.time() timestamp; while now < this, stream forward/back movement
        self._move_x_dir = 0.0     # +1.0 = forward (POINT_UP), -1.0 = backward (POINT_DOWN)
        self._move_y_until = 0.0   # time.time() timestamp; while now < this, stream left/right movement
        self._move_y_dir = 0.0     # +1.0 = right (POINT_RIGHT), -1.0 = left (POINT_LEFT)
        self._was_moving = False   # tracks movement on/off transitions, purely for the log lines below

    def cooldown_ok(self, name):
        now = time.time()
        if name not in self._last_command_time or (now - self._last_command_time[name]) > COMMAND_COOLDOWN:
            self._last_command_time[name] = now
            return True
        return False

    def _safe_send(self, description, send_fn, *args, **kwargs):
        """Wraps a MAVLink send in try/except so a dropped/broken link
        raises a logged warning instead of an unhandled exception bubbling
        out of the main loop (which would skip straight past the rest of
        this frame's telemetry handling, including the emergency-LAND path)."""
        try:
            send_fn(*args, **kwargs)
            return True
        except Exception as e:
            print(f">>> MAVLink send failed ({description}): {e}")
            return False

    def poll(self):
        """Call once per frame. All telemetry reads are non-blocking."""
        if self.state != "FLYING":
            # Guards against a stale hold marker: handle_fist() is only ever
            # called while FLYING, but clear_fist_hold() previously only
            # fired when the *gesture* changed away from FIST -- if FIST
            # ever got confirmed while NOT flying (e.g. briefly during
            # TAKING_OFF) and stayed confirmed as the vehicle transitioned
            # to FLYING, the descend-since marker could already be stale
            # by the time handle_fist() ever got called.
            self._fist_confirmed_since = None
        self._send_heartbeat_if_due()
        self._update_altitude()
        self._update_heading()
        self._update_armed_state()
        self._check_command_ack()
        self._check_status_text()
        self._check_link_health()
        self._update_state_from_telemetry()
        self._check_pending_takeoff()
        self._stream_hold_if_flying()

    def _check_link_health(self):
        # If nothing at all has come in for LINK_TIMEOUT seconds, the link
        # is almost certainly down -- flag it rather than silently keep
        # sending commands into a dead connection. This script has no way
        # to recover a real link on its own; on real hardware, treat this
        # the same as an RC/GCS failsafe and expect the vehicle's own
        # failsafe behavior (RTL/LAND) to take over independently of us.
        lost_now = (time.time() - self._last_message_time) > LINK_TIMEOUT
        if lost_now and not self.link_lost:
            print(f">>> !!! NO MAVLINK MESSAGES FOR {LINK_TIMEOUT}s -- link may be down. "
                  f"Gesture commands will keep being attempted but may never arrive.")
        self.link_lost = lost_now

    def _send_heartbeat_if_due(self):
        # ArduCopter's GCS failsafe expects to keep hearing from us --
        # without this, it can decide the GCS link is lost and autoland,
        # independent of anything the gesture code does.
        now = time.time()
        if now - self._last_heartbeat_time >= HEARTBEAT_INTERVAL:
            self._safe_send(
                "heartbeat", self.conn.mav.heartbeat_send,
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
            )
            self._last_heartbeat_time = now

    def _update_altitude(self):
        msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            self.altitude = msg.relative_alt / 1000.0
            self._last_message_time = time.time()

    def _update_heading(self):
        # VFR_HUD.heading is compass heading in degrees (0-360, 0=North),
        # already computed by the autopilot -- this is the most direct way
        # to confirm a YAW command actually turned the vehicle, independent
        # of whether COMMAND_ACK reported ACCEPTED.
        msg = self.conn.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            self.heading = msg.heading
            self._last_message_time = time.time()

    def _update_armed_state(self):
        msg = self.conn.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            self._last_message_time = time.time()
            was_armed = self.armed
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if self.armed != was_armed:
                print(f">>> Vehicle armed state changed: {'ARMED' if self.armed else 'DISARMED'}")

            new_mode = msg.custom_mode
            if new_mode != self.current_mode:
                mode_name = self.MODE_NAMES.get(new_mode, f"MODE({new_mode})")
                if self._expected_mode is not None and new_mode != self._expected_mode:
                    # The vehicle switched to a mode we did NOT command. This
                    # is the clearest possible signal of an autonomous
                    # failsafe action (commonly LAND, RTL, or SMART_RTL).
                    print(f">>> !!! VEHICLE CHANGED MODE ON ITS OWN -> {mode_name} "
                          f"(this script did not request this — check failsafe "
                          f"params: battery, RC, GCS, EKF, and GeoFence/FENCE_*)")
                else:
                    print(f">>> Vehicle mode: {mode_name}")
                self.current_mode = new_mode

    def _check_command_ack(self):
        # command_long_send() gives no guarantee of success -- this is what
        # actually surfaces a rejected ARM/TAKEOFF (e.g. failed pre-arm checks).
        msg = self.conn.recv_match(type='COMMAND_ACK', blocking=False)
        if msg:
            self._last_message_time = time.time()
            result = self.ACK_RESULT_NAMES.get(msg.result, f"UNKNOWN({msg.result})")
            cmd = self.ack_cmd_names.get(msg.command, f"CMD({msg.command})")
            print(f">>> COMMAND_ACK: {cmd} -> {result}")
            if result not in ("ACCEPTED", "IN_PROGRESS"):
                print(">>>   Rejected: check the STATUSTEXT line(s) below (or your GCS/log) for why.")

    def _check_status_text(self):
        # ArduCopter reports pre-arm failure / failsafe reasons here, e.g.
        # "PreArm: GPS not healthy" or "EKF variance".
        msg = self.conn.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            self._last_message_time = time.time()
            print(f">>> VEHICLE STATUS: {msg.text}")

    def _update_state_from_telemetry(self):
        if self.state == "TAKING_OFF" and self.altitude >= TARGET_TAKEOFF_ALT * 0.9:
            self.state = "FLYING"
            print(f">>> State: FLYING (reached {self.altitude:.1f}m)")
        elif self.state == "LANDING" and self.altitude <= 0.3:
            self.state = "ON_GROUND"
            print(">>> State: ON_GROUND (landing confirmed)")
        elif (self.state == "FLYING" and self.altitude <= TARGET_TAKEOFF_ALT * 0.5
              and time.time() >= self._descend_until):
            # Altitude dropped sharply without this script ever sending LAND,
            # AND we don't have an active FIST-driven descend window right
            # now -- that combination points to a vehicle-side failsafe
            # (GCS/RC/battery/EKF/GPS), not the gesture logic. The
            # `time.time() >= self._descend_until` guard is what keeps a
            # normal, intentional FIST descent (which legitimately walks
            # altitude below this threshold on its way to
            # MIN_DESCEND_ALTITUDE) from being misreported as a failsafe.
            print(">>> WARNING: altitude dropping while state=FLYING with no LAND "
                  "command sent by this script. Check vehicle failsafe params "
                  "(FS_GCS_ENABLE/TIMEOUT, FS_THR_ENABLE, EKF/GPS/battery failsafes) "
                  "and the autopilot's own log.")
            self.state = "LANDING"

    def _check_pending_takeoff(self):
        if self._pending_takeoff_time is not None and time.time() >= self._pending_takeoff_time:
            self._safe_send(
                "TAKEOFF", self.conn.mav.command_long_send,
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TARGET_TAKEOFF_ALT
            )
            print(">>> TAKEOFF command sent")
            self._pending_takeoff_time = None

    def _stream_hold_if_flying(self):
        # While airborne, continuously re-send a GUIDED velocity setpoint at
        # HOLD_STREAM_INTERVAL -- this is what keeps the vehicle in GUIDED
        # (no RC input, no mode switch, so OPEN_PALM can no longer trigger
        # an ALT_HOLD-with-no-throttle auto-disarm) instead of a one-shot
        # command that goes stale.
        #
        # Normally that setpoint is zero-velocity (hold position). But a
        # single one-shot climb/descend pulse would get overwritten by the
        # very next hold tick within HOLD_STREAM_INTERVAL (<=0.25s), which
        # is why an early version of this script wasn't visibly gaining
        # altitude -- the climb was being cancelled almost as soon as it
        # started. So instead: continuous_ascend()/handle_fist() are called
        # every frame their gesture is held and keep pushing a short timed
        # window (_climb_until / _descend_until) forward; *this* stream
        # sends the climb/descend velocity for as long as the relevant
        # window stays current, and falls back to zero-velocity hold within
        # CLIMB_HOLD_GRACE of the gesture (or the hand) being released.
        # Climb takes priority over descend if both windows were somehow
        # active at once (they shouldn't be -- see continuous_ascend/handle_fist).
        #
        # yaw() used to send a one-shot MAV_CMD_CONDITION_YAW command on a
        # separate MAVLink message path from this velocity stream. ArduCopter's
        # GUIDED yaw controller treats each fresh velocity-only setpoint
        # arriving from this stream as a cue to fall back to holding the
        # current heading -- so a turn-in-progress kept getting interrupted
        # every HOLD_STREAM_INTERVAL, which is why it crept instead of
        # turning and never completed a full rotation. Fix: same pattern as
        # climb -- yaw() arms a timed window (_yaw_until) and a yaw *rate*
        # rides along in this same setpoint, so there's no second competing
        # command path to interrupt it.
        if self.state != "FLYING":
            return
        now = time.time()
        if now - self._last_hold_stream_time >= HOLD_STREAM_INTERVAL:
            is_climbing = now < self._climb_until
            is_descending = (not is_climbing) and now < self._descend_until

            if is_climbing and not self._was_climbing:
                print(">>> ASCEND: climbing while ROCK_ON is held")
            elif self._was_climbing and not is_climbing:
                print(">>> ASCEND: gesture released -- holding altitude")
            self._was_climbing = is_climbing

            if is_descending and not self._was_descending:
                print(">>> DESCEND: descending while FIST is held (above landing floor)")
            elif self._was_descending and not is_descending:
                print(">>> DESCEND: stopped -- holding altitude")
            self._was_descending = is_descending

            is_moving_x = now < self._move_x_until
            is_moving_y = (not is_moving_x) and now < self._move_y_until
            # Forward/back takes priority over strafe if both windows were
            # somehow active at once -- mirrors the climb-over-descend
            # priority above. In practice this shouldn't happen since
            # confirm() only ever hands the main loop one gesture at a time.
            is_moving = is_moving_x or is_moving_y

            if is_moving and not self._was_moving:
                print(">>> MOVE: horizontal movement gesture held")
            elif self._was_moving and not is_moving:
                print(">>> MOVE: gesture released -- holding position")
            self._was_moving = is_moving

            vx = MOVE_SPEED * self._move_x_dir if is_moving_x else 0.0
            vy = MOVE_SPEED * self._move_y_dir if is_moving_y else 0.0

            vz = -CLIMB_RATE if is_climbing else (DESCEND_RATE if is_descending else 0.0)
            yaw_rate = self._active_yaw_rate if now < self._yaw_until else None
            self._send_velocity_setpoint(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)
            self._last_hold_stream_time = now

    def _send_velocity_setpoint(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=None):
        # yaw_rate=None means "don't touch yaw at all" (ignore both yaw and
        # yaw_rate fields) so this setpoint can't interrupt an unrelated yaw
        # in progress. Pass an explicit yaw_rate (rad/s, can be 0.0) to take
        # control of the yaw rate for this packet.
        if yaw_rate is None:
            type_mask = 0b0000111111000111  # ignore pos, accel, yaw, yaw_rate; control velocity
            yaw_rate_field = 0.0
        else:
            type_mask = 0b0000011111000111  # ignore pos, accel, yaw; control velocity AND yaw_rate
            yaw_rate_field = yaw_rate
        self._safe_send(
            "velocity setpoint", self.conn.mav.set_position_target_local_ned_send,
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate_field
        )


    # ---- commands ----
    def arm_and_takeoff(self):
        """Sends ARM immediately, queues TAKEOFF to fire after a short delay
        (non-blocking -- no time.sleep in the main loop)."""
        self._safe_send(
            "set mode GUIDED", self.conn.mav.set_mode_send,
            self.conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_GUIDED
        )
        self._expected_mode = MODE_GUIDED
        self._safe_send(
            "ARM", self.conn.mav.command_long_send,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )
        self._pending_takeoff_time = time.time() + TAKEOFF_ARM_DELAY
        self.state = "TAKING_OFF"
        print(">>> ARM command sent, takeoff queued")

    def land(self):
        # MAV_CMD_NAV_LAND makes ArduCopter switch itself into LAND mode as
        # a normal side effect -- that's expected, not a failsafe. Update
        # _expected_mode here so the mode-watcher in _update_armed_state()
        # doesn't misreport this intentional switch as "VEHICLE CHANGED
        # MODE ON ITS OWN" (it was previously left at whatever mode --
        # e.g. GUIDED -- was expected before land() was ever called).
        self._expected_mode = MODE_LAND
        self._safe_send(
            "LAND", self.conn.mav.command_long_send,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
        )
        self.state = "LANDING"
        print(">>> LAND command sent")

    def hover(self):
        """OPEN_PALM handler. We stay in GUIDED -- no mode switch here.
        Position/altitude hold is already being streamed continuously by
        _stream_hold_if_flying() every tick while FLYING, so this just
        gives immediate feedback and (re)confirms we're not expecting any
        other mode."""
        self._expected_mode = MODE_GUIDED
        self._climb_until = 0.0    # cancel any in-progress climb; go straight to zero-velocity hold
        self._descend_until = 0.0  # cancel any in-progress descend too
        self._yaw_until = 0.0      # cancel any in-progress yaw too
        self._move_x_until = 0.0   # cancel any in-progress forward/back movement too
        self._move_y_until = 0.0   # cancel any in-progress strafe too
        self._fist_confirmed_since = None
        self._send_velocity_setpoint(vz=0.0)
        print(">>> HOVER confirmed (GUIDED position hold)")

    def continuous_ascend(self):
        """Call every frame while ROCK_ON is the confirmed gesture and the
        vehicle is FLYING. Pushes the climb window (_climb_until) forward by
        CLIMB_HOLD_GRACE rather than arming a fixed-length burst, so the
        vehicle keeps climbing for exactly as long as the gesture is held
        and glides back to a hold within CLIMB_HOLD_GRACE of release (see
        the comment in _stream_hold_if_flying). Refuses to extend the window
        once at/above MAX_ALTITUDE. No cooldown here -- this isn't a
        discrete one-shot command, it's a continuous input, so it's safe
        (and correct) to call it every single frame the gesture holds."""
        if self.altitude >= MAX_ALTITUDE:
            return
        self._descend_until = 0.0  # climb always wins if both were somehow requested
        self._climb_until = time.time() + CLIMB_HOLD_GRACE

    def move(self, vx_dir=0.0, vy_dir=0.0):
        """Call every frame a horizontal movement gesture (POINT_UP,
        POINT_DOWN, POINT_LEFT, POINT_RIGHT) is confirmed and the vehicle
        is FLYING. Same windowed-push pattern as continuous_ascend(): each
        call pushes the relevant axis's window (_move_x_until for forward/
        back, _move_y_until for left/right) forward by MOVE_HOLD_GRACE, so
        movement continues for exactly as long as the gesture is held and
        glides back to a hold within MOVE_HOLD_GRACE of release. Pass
        vx_dir=+1.0 for forward, -1.0 for backward, vy_dir=+1.0 for right,
        -1.0 for left (body-frame axes -- see MAV_FRAME_BODY_OFFSET_NED in
        _send_velocity_setpoint). No altitude or geofence cap here, unlike
        continuous_ascend()/handle_fist() -- this script has no positional
        awareness of obstacles or boundaries, so horizontal movement is
        only as safe as the open space actually in front of/around the
        vehicle. No cooldown -- continuous input, safe to call every frame
        the gesture holds."""
        if vx_dir != 0.0:
            self._move_x_dir = vx_dir
            self._move_x_until = time.time() + MOVE_HOLD_GRACE
        if vy_dir != 0.0:
            self._move_y_dir = vy_dir
            self._move_y_until = time.time() + MOVE_HOLD_GRACE

    def handle_fist(self):
        """Call every frame FIST is the confirmed gesture and the vehicle is
        FLYING. FIST continuously descends the vehicle (same windowed-push
        pattern as continuous_ascend(), just downward) for as long as it's
        held. Once altitude reaches MIN_DESCEND_ALTITUDE, this fires the
        real LAND command instead and lets ArduCopter's own landing
        controller take it the rest of the way down -- gesture control
        never tries to touch down on its own.

        Note there's no time-based cap on this (see the v6 fix note at the
        top of the file): an unreleased FIST will ride the vehicle all the
        way to the landing floor and then auto-land. Release the gesture
        once you're at the altitude you want.

        clear_fist_hold() must be called whenever FIST stops being the
        confirmed gesture, so the display-only hold marker doesn't go stale."""
        if self.altitude <= MIN_DESCEND_ALTITUDE:
            self.land()
            self._fist_confirmed_since = None
            return

        if self._fist_confirmed_since is None:
            self._fist_confirmed_since = time.time()

        self._climb_until = 0.0  # descend always wins over any leftover climb window
        self._descend_until = time.time() + CLIMB_HOLD_GRACE

    def fist_hold_progress(self):
        """Returns (current_altitude, floor_altitude) while FIST is
        currently being held and driving a descent, or None otherwise.
        Display-only -- lets you see how much altitude remains before FIST
        triggers LAND, instead of guessing."""
        if self._fist_confirmed_since is None:
            return None
        return (self.altitude, MIN_DESCEND_ALTITUDE)

    def clear_fist_hold(self):
        """Call whenever the confirmed gesture is NOT FIST (or no hand is
        visible) so the display-only hold marker doesn't linger stale."""
        self._fist_confirmed_since = None

    def yaw(self, direction):
        # Arms a YAW_DURATION-second window during which
        # _stream_hold_if_flying() rides a yaw_rate command along with the
        # regular hold/climb/descend setpoint, instead of firing a separate
        # one-shot MAV_CMD_CONDITION_YAW that the hold stream would
        # interrupt every HOLD_STREAM_INTERVAL (see the comment in
        # _stream_hold_if_flying).
        self._yaw_until = time.time() + YAW_DURATION
        self._active_yaw_rate = math.radians(YAW_RATE_DPS) * direction
        heading_str = f"{self.heading}°" if self.heading is not None else "unknown"
        self._send_velocity_setpoint(yaw_rate=self._active_yaw_rate)
        print(f">>> YAW {'RIGHT' if direction == 1 else 'LEFT'} command sent "
              f"(heading was {heading_str}, turning for {YAW_DURATION}s at {YAW_RATE_DPS} deg/s "
              f"-- watch the Heading readout, and for a 'COMMAND_ACK: YAW -> ACCEPTED' line)")


# ======================= Camera =======================

def open_camera(index=0):
    """CAP_DSHOW tends to be more reliable for sustained capture on Windows
    than CAP_MSMF for many USB webcams; falls back to MSMF if that fails."""
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("Could not open camera with CAP_DSHOW, trying CAP_MSMF...")
        cap.release()
        cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
    if not cap.isOpened():
        print(f"ERROR: Camera index {index} failed to open with both DSHOW and MSMF backends.")
        print("Close any other app using the camera (Zoom/Teams/Camera app/browser),")
        print("check for a leftover python.exe process holding it, or try a different")
        print("--camera-index, and try again.")
        cap.release()
        return None
    return cap


# ======================= CLI =======================

def parse_args():
    parser = argparse.ArgumentParser(description="Gesture-controlled drone via webcam + MediaPipe + MAVLink")
    parser.add_argument('--connect', default=MAVLINK_CONN,
                         help=f"MAVLink connection string (default: {MAVLINK_CONN})")
    parser.add_argument('--model', default=MODEL_PATH,
                         help=f"Path to the MediaPipe hand_landmarker .task model file (default: {MODEL_PATH})")
    parser.add_argument('--camera-index', type=int, default=0,
                         help="OpenCV camera index (default: 0)")
    return parser.parse_args()


# ======================= Main =======================

def main():
    args = parse_args()

    if not os.path.isfile(args.model):
        print(f"ERROR: MediaPipe model file not found at '{args.model}'.")
        print("Download hand_landmarker.task from Google's MediaPipe model zoo")
        print("(https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker),")
        print("or pass the correct path with --model.")
        return

    base_options = python.BaseOptions(model_asset_path=args.model)
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        num_hands=1,
        min_hand_detection_confidence=0.7,
        min_tracking_confidence=0.5,
    )
    detector = vision.HandLandmarker.create_from_options(options)

    try:
        drone = DroneController(args.connect)
    except RuntimeError as e:
        print(f"ERROR: {e}")
        detector.close()
        return

    recognizer = GestureRecognizer()

    cap = open_camera(args.camera_index)
    if cap is None:
        detector.close()
        return

    print("Press 'q' to quit")
    consecutive_failures = 0
    MAX_FAILURES = 30  # ~1s of dropped frames at 30fps before giving up

    try:
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                consecutive_failures += 1
                print(f"Camera read failed ({consecutive_failures}/{MAX_FAILURES})")
                if consecutive_failures >= MAX_FAILURES:
                    print("Camera unrecoverable — exiting loop")
                    break
                time.sleep(0.03)
                continue
            consecutive_failures = 0

            drone.poll()

            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            result = detector.detect(mp_image)

            raw_gesture, gesture, finger_states = "NONE", "NONE", None

            if result.hand_landmarks:
                for lm in result.hand_landmarks:
                    h, w, _ = frame.shape
                    for p in lm:
                        cv2.circle(frame, (int(p.x * w), int(p.y * h)), 4, (0, 255, 0), -1)

                    raw_gesture, finger_states = recognizer.classify(lm)
                    gesture = recognizer.confirm(raw_gesture)

                    if gesture != "FIST":
                        drone.clear_fist_hold()

                    if gesture == "THUMBS_UP" and drone.state == "ON_GROUND" and drone.cooldown_ok("takeoff"):
                        drone.arm_and_takeoff()

                    elif gesture == "FIST" and drone.state == "FLYING":
                        # Continuous input (descend until the landing floor,
                        # then auto-land) -- called every frame the gesture
                        # is held, no cooldown gate (see handle_fist()'s
                        # docstring).
                        drone.handle_fist()

                    elif gesture == "OPEN_PALM" and drone.state == "FLYING" and drone.cooldown_ok("hover"):
                        drone.hover()

                    elif gesture == "ROCK_ON" and drone.state == "FLYING":
                        # Continuous input, not a discrete command -- called every
                        # frame the gesture is held, no cooldown gate (see
                        # continuous_ascend()'s docstring).
                        drone.continuous_ascend()

                    elif gesture == "THUMBS_RIGHT" and drone.state == "FLYING" and drone.cooldown_ok("yaw"):
                        drone.yaw(1)

                    elif gesture == "THUMBS_LEFT" and drone.state == "FLYING" and drone.cooldown_ok("yaw"):
                        drone.yaw(-1)

                    elif gesture == "PEACE_SIGN" and drone.state == "FLYING" and drone.cooldown_ok("land_now"):
                        # Discrete override: skip the FIST descend-to-floor
                        # ramp and land immediately from whatever altitude
                        # we're at right now.
                        drone.land()

                    elif gesture == "POINT_UP" and drone.state == "FLYING":
                        # Continuous input, no cooldown gate -- same pattern
                        # as continuous_ascend()/handle_fist() (see move()).
                        drone.move(vx_dir=1.0)

                    elif gesture == "POINT_DOWN" and drone.state == "FLYING":
                        drone.move(vx_dir=-1.0)

                    elif gesture == "POINT_LEFT" and drone.state == "FLYING":
                        drone.move(vy_dir=-1.0)

                    elif gesture == "POINT_RIGHT" and drone.state == "FLYING":
                        drone.move(vy_dir=1.0)
            else:
                recognizer.reset()
                drone.clear_fist_hold()

            cv2.putText(frame, f"Raw: {raw_gesture}  Confirmed: {gesture}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(frame, f"State: {drone.state}  Armed: {drone.armed}  "
                                f"Mode: {DroneController.MODE_NAMES.get(drone.current_mode, drone.current_mode)}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(frame, f"Alt: {drone.altitude:.1f}m  "
                                f"Heading: {drone.heading if drone.heading is not None else '--'}deg",
                        (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            if drone.link_lost:
                cv2.putText(frame, "LINK LOST", (10, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            fist_progress = drone.fist_hold_progress()
            if fist_progress is not None:
                alt, floor = fist_progress
                cv2.putText(frame, f"FIST descending: {alt:.1f}m -> LAND at {floor:.1f}m",
                            (10, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)
            if finger_states is not None:
                fs = finger_states
                cv2.putText(
                    frame,
                    f"T:{int(fs['thumb'])} I:{int(fs['index'])} M:{int(fs['middle'])} "
                    f"R:{int(fs['ring'])} P:{int(fs['pinky'])}",
                    (10, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                )
            move_labels = {
                "POINT_UP": "MOVING FORWARD", "POINT_DOWN": "MOVING BACKWARD",
                "POINT_LEFT": "STRAFING LEFT", "POINT_RIGHT": "STRAFING RIGHT",
            }
            if gesture in move_labels and drone.state == "FLYING":
                cv2.putText(frame, move_labels[gesture], (10, 270),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Gesture Drone Control', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Safety failsafe: if we're exiting the loop (crash, camera loss, or
        # quit key) while airborne, send LAND so the drone isn't stranded
        # in the air with no further control input.
        if drone.state in ("FLYING", "TAKING_OFF"):
            print(">>> Loop ending while airborne — sending emergency LAND")
            try:
                drone.land()
            except Exception as e:
                print(f"Failed to send emergency land command: {e}")

        # Defensive teardown: don't let a native exception on release()
        # skip cleanup of the window or the MediaPipe detector.
        try:
            if cap.isOpened():
                cap.release()
        except cv2.error as e:
            print(f"cap.release() raised (ignored): {e}")

        cv2.destroyAllWindows()
        detector.close()


if __name__ == "__main__":
    main()
