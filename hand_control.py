"""
hand_control.py
Turns hand movement into arm targets. No camera or network code in here, so it
can be tested on its own.

Works like a computer mouse with a "clutch":
  * When your hand appears (and stays for ENGAGE_TIME), its position is locked
    in as the reference point. The arm does NOT jump to your hand.
  * From then on, moving your hand moves the arm by the same kind of amount
    (relative control), like moving a mouse.
  * Make a fist (or take your hand away) to let go. The arm holds still, and
    you can move your hand back to a comfortable spot and open it again,
    like lifting a mouse off the desk.
  * Pinch thumb and index finger together to close the pincher.

Axes (the camera faces you, the preview is mirrored like a selfie):
  hand left / right         ->  base rotation (theta)
  hand up / down            ->  height (z)
  hand towards / away from camera  ->  reach (r)
  tilt hand (optional)      ->  hand roll
  pinch                     ->  pincher
"""

import copy
import math
from dataclasses import dataclass

from hand_features import OneEuroFilter

# =========================================================
# MODE
#   "absolute": the arm copies where your hand is in the picture.
#               Hand in the middle = arm in the middle of its box below.
#   "relative": mouse-style, hand movement nudges the arm from where it is.
#   "follow":   camera mounted ON the arm. The arm moves to keep your hand in
#               the centre of the picture, at the distance it was when it
#               first saw you.
# =========================================================

MODE = "follow"

# ---- Follow mode (camera on the arm) ----
# Error = how far the hand is from the centre (x, y go from -0.5 to +0.5).
# The arm moves at a speed proportional to the error, capped by the *_RATE
# values further down. Flip a FOLLOW_SIGN_ if the arm moves AWAY from your hand.
FOLLOW_K_THETA = 0.6      # rad/s of base turn per unit of left/right error
FOLLOW_K_Z     = 200.0    # mm/s of height per unit of up/down error
FOLLOW_K_R     = 150.0    # mm/s of reach per unit of distance error (size too small/big)
FOLLOW_DEADBAND_XY   = 0.06   # ignore the hand being this close to centre (stops wobbling)
FOLLOW_DEADBAND_SIZE = 0.15   # ignore size changes smaller than 15%
FOLLOW_SIGN_THETA = -1    # hand right of centre -> turn base right
FOLLOW_SIGN_Z     = +1    # hand above centre -> move up
FOLLOW_SIGN_R     = +1    # hand looks smaller (further) -> reach towards it
FOLLOW_DISTANCE   = True  # set False to only follow left/right/up/down

# ---- Absolute mode: map the picture onto a box the arm can reach ----
# Hand across the picture (0 = left edge, 1 = right edge, mirrored like a selfie)
X_LEFT, X_RIGHT   = 0.20, 0.80
THETA_SPAN_DEG    = 50.0     # base turns +/- this many degrees edge to edge
# Hand up/down in the picture (0 = top, 1 = bottom)
Y_TOP, Y_BOTTOM   = 0.20, 0.80
Z_AT_TOP          = 500.0    # arm height (mm) when your hand is at the top
Z_AT_BOTTOM       = 150.0    # ... at the bottom.   Swap these two if up/down is backwards
# Hand distance, measured by how big your palm looks ("size" on the preview)
SIZE_FAR, SIZE_NEAR = 0.09, 0.20   # palm size far from / close to the camera
R_AT_FAR          = 200.0    # arm reach (mm) when your hand is far from the camera
R_AT_NEAR         = 500.0    # ... close to the camera. Swap these two if forward/back is backwards

# =========================================================
# Relative mode tuning. Flip a SIGN_ value if an axis moves the wrong way.
# =========================================================

THETA_GAIN = 0.8      # radians of base rotation for moving across the whole image (was 1.6)
Z_GAIN     = 200.0    # mm of height for moving from the top to the bottom of the image (was 450)
R_GAIN     = 150.0    # mm of reach for the palm looking 2x bigger (was 350)
ROLL_GAIN  = 1.0      # degrees of arm roll per degree of hand tilt

SIGN_THETA = -1       # hand to your right -> which way the base turns (both modes)
SIGN_Z     = -1       # hand up -> arm up (was +1, arm went the wrong way)
SIGN_R     = +1       # hand towards camera -> arm reaches further
SIGN_ROLL  = -1

ENABLE_ROLL = False   # turn on once the other axes feel good

ENGAGE_TIME = 0.5     # s the hand must be seen before the arm follows it
LOST_GRACE  = 0.3     # s the hand can drop out before we let go

# Same limits and max speeds as servo_controller.py
R_MIN, R_MAX         = 50.0, 900.0
Z_MIN, Z_MAX         = 0.0, 900.0
PITCH_MIN, PITCH_MAX = -90.0, 90.0
ROLL_MIN, ROLL_MAX   = -180.0, 180.0

R_RATE     = 80.0     # mm/s       still slow - raise once you trust it (original 250)
THETA_RATE = 0.35     # rad/s (~20 deg/s)  (original 0.8)
Z_RATE     = 80.0     # mm/s       (original 200)
ROLL_RATE  = 15.0     # deg/s      SLOW START (was 90)

# Pincher: pinch ratio -> servo position
PINCHER_MIN   = 1000.0   # fully open
PINCHER_MAX   = 2200.0   # closed + grip
PINCH_CLOSED  = 0.25     # thumb-index gap / palm size when touching
PINCH_OPEN    = 0.90     # ... when wide open


@dataclass
class ArmPose:
    """Arm target in the same cylindrical form servo_controller.py uses."""
    r: float = 300.0          # mm out from the base
    theta: float = 0.0        # rad around the base
    z: float = 350.0          # mm up
    pitch: float = -20.0      # deg
    yaw_offset: float = 0.0   # deg on top of theta
    roll: float = 0.0         # deg

    def to_ik(self, speed):
        """JSON body for POST /ik on full_servo_hud.py."""
        return {
            "X": round(self.r * math.cos(self.theta), 1),
            "Y": round(self.r * math.sin(self.theta), 1),
            "Z": round(self.z, 1),
            "Pitch": round(self.pitch, 2),
            "Yaw": round(math.degrees(self.theta) + self.yaw_offset, 2),
            "Roll": round(self.roll, 2),
            "Speed": speed,
        }

    @classmethod
    def from_ik(cls, t):
        """Build from the "target" dict that GET /state returns."""
        x, y = float(t["X"]), float(t["Y"])
        theta = math.atan2(y, x)
        return cls(
            r=math.hypot(x, y), theta=theta, z=float(t["Z"]),
            pitch=float(t.get("Pitch", -20.0)),
            yaw_offset=float(t.get("Yaw", math.degrees(theta))) - math.degrees(theta),
            roll=float(t.get("Roll", 0.0)),
        )


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _map(v, v0, v1, out0, out1):
    """Linear map v from [v0, v1] to [out0, out1], clamped at the ends."""
    t = _clamp((v - v0) / (v1 - v0), 0.0, 1.0)
    return out0 + t * (out1 - out0)


def _deadband(e, band):
    """Zero inside the band, then grows from zero (no jump at the edge)."""
    if abs(e) <= band:
        return 0.0
    return e - band if e > 0 else e + band


def _step(current, target, max_step):
    return current + _clamp(target - current, -max_step, max_step)


def pinch_to_pincher(pinch):
    """Map the pinch ratio to a pincher position (small gap = closed)."""
    t = (pinch - PINCH_CLOSED) / (PINCH_OPEN - PINCH_CLOSED)
    t = _clamp(t, 0.0, 1.0)
    return PINCHER_MAX - t * (PINCHER_MAX - PINCHER_MIN)


class HandArmController:
    def __init__(self, start_pose=None):
        # Last pose the server accepted (in reach). Everything is relative to it.
        self.pose = start_pose or ArmPose()
        # Last pose we asked for (may not be confirmed yet). Used for speed limiting.
        self._cmd = copy.copy(self.pose)
        self.state = "no hand"
        self.debug = ""
        self._anchor_hand = None
        self._anchor_pose = None
        self._seen_since = None
        self._last_seen = None
        self._last_t = None
        self._filters = {
            "x":     OneEuroFilter(min_cutoff=1.0, beta=3.0),
            "y":     OneEuroFilter(min_cutoff=1.0, beta=3.0),
            "size":  OneEuroFilter(min_cutoff=0.8, beta=10.0),
            "roll":  OneEuroFilter(min_cutoff=1.0, beta=0.05),
            "pinch": OneEuroFilter(min_cutoff=2.0, beta=1.0),
        }

    # -----------------------------------------------------
    def release(self):
        """Let go of the hand: arm holds, next hand re-anchors."""
        self._anchor_hand = None
        self._anchor_pose = None
        self._seen_since = None
        for f in self._filters.values():
            f.reset()

    def confirm(self, pose):
        """Call when the server says a pose was in reach."""
        self.pose = copy.copy(pose)

    def reject(self):
        """Call when the server says a pose was out of reach: fall back to the last good one."""
        self._cmd = copy.copy(self.pose)

    # -----------------------------------------------------
    def update(self, feat, now):
        """
        feat: HandFeatures or None if no hand in this frame.
        Returns (pose_to_send or None, pincher_to_send or None).
        """
        dt = 0.0 if self._last_t is None else min(now - self._last_t, 0.2)
        self._last_t = now

        if feat is None:
            if self._last_seen is None or now - self._last_seen > LOST_GRACE:
                if self._anchor_hand is not None or self._seen_since is not None:
                    self.release()
                self.state = "no hand"
            return None, None
        self._last_seen = now

        # Smooth everything
        sx = self._filters["x"](feat.x, now)
        sy = self._filters["y"](feat.y, now)
        ss = self._filters["size"](feat.size, now)
        sr = self._filters["roll"](feat.roll, now)
        sp = self._filters["pinch"](feat.pinch, now)

        if feat.fist:
            if self._anchor_hand is not None:
                self.release()
            self.state = "paused (fist)"
            return None, None

        if self._seen_since is None:
            self._seen_since = now
        if now - self._seen_since < ENGAGE_TIME:
            self.state = "hand found, hold still..."
            return None, None

        if self._anchor_hand is None:
            self._anchor_hand = (sx, sy, ss, sr)
            self._anchor_pose = copy.copy(self.pose)
            self._cmd = copy.copy(self.pose)

        x0, y0, s0, r0 = self._anchor_hand
        a = self._anchor_pose

        if MODE == "follow":
            ex = _deadband(sx - 0.5, FOLLOW_DEADBAND_XY)          # + = hand right of centre
            ey = _deadband(sy - 0.5, FOLLOW_DEADBAND_XY)          # + = hand below centre
            es = _deadband((s0 - ss) / s0, FOLLOW_DEADBAND_SIZE)  # + = hand looks smaller (further away)

            v_theta = _clamp(FOLLOW_SIGN_THETA * FOLLOW_K_THETA * ex, -THETA_RATE, THETA_RATE)
            v_z     = _clamp(FOLLOW_SIGN_Z * FOLLOW_K_Z * -ey,        -Z_RATE, Z_RATE)
            v_r     = _clamp(FOLLOW_SIGN_R * FOLLOW_K_R * es,         -R_RATE, R_RATE)
            if not FOLLOW_DISTANCE:
                v_r = 0.0

            c = self._cmd
            out = copy.copy(c)
            out.theta = c.theta + v_theta * dt
            out.z     = _clamp(c.z + v_z * dt, Z_MIN, Z_MAX)
            out.r     = _clamp(c.r + v_r * dt, R_MIN, R_MAX)
            self._cmd = out

            self.state = "tracking"
            self.debug = (f"off-centre x={sx - 0.5:+.2f} y={sy - 0.5:+.2f} size={ss:.3f} (keep {s0:.3f})  "
                          f"-> base {math.degrees(v_theta):+.0f}deg/s z {v_z:+.0f} r {v_r:+.0f} mm/s")
            return out, pinch_to_pincher(sp)

        # Where the hand says the arm should be
        want = copy.copy(a)
        if MODE == "absolute":
            want.theta = math.radians(SIGN_THETA * _map(sx, X_LEFT, X_RIGHT,
                                                        -THETA_SPAN_DEG, THETA_SPAN_DEG))
            want.z = _map(sy, Y_TOP, Y_BOTTOM, Z_AT_TOP, Z_AT_BOTTOM)
            want.r = _map(ss, SIZE_FAR, SIZE_NEAR, R_AT_FAR, R_AT_NEAR)
        else:
            want.theta = a.theta + SIGN_THETA * (sx - x0) * THETA_GAIN
            want.z     = a.z     + SIGN_Z * (y0 - sy) * Z_GAIN
            want.r     = a.r     + SIGN_R * (ss / s0 - 1.0) * R_GAIN
        if ENABLE_ROLL:
            want.roll = a.roll + SIGN_ROLL * (sr - r0) * ROLL_GAIN

        want.r     = _clamp(want.r, R_MIN, R_MAX)
        want.z     = _clamp(want.z, Z_MIN, Z_MAX)
        want.roll  = _clamp(want.roll, ROLL_MIN, ROLL_MAX)
        want.pitch = _clamp(want.pitch, PITCH_MIN, PITCH_MAX)

        # Speed limit, measured from the last pose we sent
        c = self._cmd
        out = copy.copy(c)
        out.r     = _step(c.r,     want.r,     R_RATE * dt)
        out.theta = _step(c.theta, want.theta, THETA_RATE * dt)
        out.z     = _step(c.z,     want.z,     Z_RATE * dt)
        out.roll  = _step(c.roll,  want.roll,  ROLL_RATE * dt)
        self._cmd = out

        self.state = "tracking"
        self.debug = (f"hand x={sx:.2f} y={sy:.2f} size={ss:.3f}  ->  "
                      f"target r={want.r:.0f} base={math.degrees(want.theta):.0f}deg z={want.z:.0f}")
        return out, pinch_to_pincher(sp)
