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
# Tuning. Flip a SIGN_ value if an axis moves the wrong way.
# =========================================================

THETA_GAIN = 1.6      # radians of base rotation for moving across the whole image
Z_GAIN     = 450.0    # mm of height for moving from the top to the bottom of the image
R_GAIN     = 350.0    # mm of reach for the palm looking 2x bigger (hand ~2x closer)
ROLL_GAIN  = 1.0      # degrees of arm roll per degree of hand tilt

SIGN_THETA = -1       # hand to your right -> which way the base turns
SIGN_Z     = +1       # hand up -> arm up
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

R_RATE     = 250.0    # mm/s
THETA_RATE = 0.8      # rad/s
Z_RATE     = 200.0    # mm/s
ROLL_RATE  = 90.0     # deg/s

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

        # Where the hand says the arm should be
        want = copy.copy(a)
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
        return out, pinch_to_pincher(sp)
