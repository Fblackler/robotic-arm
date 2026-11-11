"""
hand_control.py
Decides how the arm moves based on where the hand is in the camera picture.
No camera or network code in here, so it can be tested on its own.

The camera is mounted on the arm. Each frame:
  * work out which direction the hand is from the centre of the picture
  * inside the dead zone (centre circle): hold still
  * outside it: move the arm towards the hand at one fixed, slow speed
      hand left in the picture -> arm moves right (the camera image is flipped)
      hand up in the picture   -> arm moves up (whole arm, Z)
  * fist, or no hand: stop
  * pinch thumb + index finger: close the gripper
"""

import copy
import math
from dataclasses import dataclass

from hand_features import OneEuroFilter

# =========================================================
# Settings
# =========================================================

SPEED       = 30.0    # mm/s, fixed
DEADZONE    = 0.10    # radius of the centre dead zone, as a fraction of picture height
HYSTERESIS  = 0.03    # start moving at DEADZONE+this, stop at DEADZONE-this (stops flicker)
SIGN_H      = +1      # flip if left/right goes the wrong way
SIGN_V      = -1      # flip if up/down goes the wrong way (-1 is correct on our arm)

ENGAGE_TIME = 0.5     # s the hand must be seen before the arm follows it
LOST_GRACE  = 0.3     # s the hand can drop out of view before we reset

Z_MIN, Z_MAX = 0.0, 900.0   # height limits (mm)

# Gripper: pinch ratio -> servo position
PINCHER_MIN  = 1000.0   # fully open
PINCHER_MAX  = 2200.0   # closed + grip
PINCH_CLOSED = 0.25     # thumb-index gap / palm size when touching
PINCH_OPEN   = 0.90     # ... when wide open


# =========================================================
# Arm target
# =========================================================

@dataclass
class ArmPose:
    """Arm target in cylindrical coordinates, same as servo_controller.py."""
    r: float = 300.0          # mm out from the base
    theta: float = 0.0        # rad around the base
    z: float = 350.0          # mm up
    pitch: float = -20.0      # deg
    yaw_offset: float = 0.0   # deg on top of theta
    roll: float = 0.0         # deg

    def to_ik(self, speed):
        """JSON body for POST /ik on the server."""
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


def pinch_to_pincher(pinch):
    """Map the pinch ratio to a gripper position (small gap = closed)."""
    t = _clamp((pinch - PINCH_CLOSED) / (PINCH_OPEN - PINCH_CLOSED), 0.0, 1.0)
    return PINCHER_MAX - t * (PINCHER_MAX - PINCHER_MIN)


# =========================================================
# Controller
# =========================================================

class HandArmController:
    def __init__(self, start_pose=None):
        self.pose = start_pose or ArmPose()   # last pose the server accepted
        self._cmd = copy.copy(self.pose)      # last pose we asked for
        self.aspect = 640 / 480               # picture width / height (set by hand_tracker)
        self.state = "no hand"
        self.debug = ""
        self._moving = False
        self._seen_since = None
        self._last_seen = None
        self._last_t = None
        self._filters = {
            "x":     OneEuroFilter(min_cutoff=1.0, beta=3.0),
            "y":     OneEuroFilter(min_cutoff=1.0, beta=3.0),
            "pinch": OneEuroFilter(min_cutoff=2.0, beta=1.0),
        }

    def release(self):
        """Forget the hand: the arm holds still until a hand is seen again."""
        self._seen_since = None
        self._moving = False
        for f in self._filters.values():
            f.reset()

    def confirm(self, pose):
        """The server said this pose was in reach."""
        self.pose = copy.copy(pose)

    def reject(self):
        """The server said the last pose was out of reach: go back to the last good one."""
        self._cmd = copy.copy(self.pose)

    def update(self, feat, now):
        """
        feat: HandFeatures, or None if no hand in this frame.
        Returns (pose_to_send or None, gripper_position or None).
        """
        dt = 0.0 if self._last_t is None else min(now - self._last_t, 0.2)
        self._last_t = now

        # No hand: hold still, and reset once it's been gone a moment
        if feat is None:
            if self._last_seen is None or now - self._last_seen > LOST_GRACE:
                self.release()
                self.state = "no hand"
            return None, None
        self._last_seen = now

        # Smooth out jitter
        sx = self._filters["x"](feat.x, now)
        sy = self._filters["y"](feat.y, now)
        sp = self._filters["pinch"](feat.pinch, now)

        # Fist: stop
        if feat.fist:
            self.release()
            self.state = "paused (fist)"
            return None, None

        # Hand must be seen for a moment before we follow it
        if self._seen_since is None:
            self._seen_since = now
            self._cmd = copy.copy(self.pose)
        if now - self._seen_since < ENGAGE_TIME:
            self.state = "hand found, hold still..."
            return None, None

        # Where is the hand relative to the centre of the picture?
        dx = (sx - 0.5) * self.aspect      # + = right of centre (scaled so x and y use the same units)
        dy = sy - 0.5                      # + = below centre
        dist = math.hypot(dx, dy)
        angle = math.degrees(math.atan2(-dy, dx))   # 0 = right, 90 = up

        # Dead zone with hysteresis so it doesn't flick on/off at the edge
        if self._moving and dist < DEADZONE - HYSTERESIS:
            self._moving = False
        elif not self._moving and dist > DEADZONE + HYSTERESIS:
            self._moving = True

        c = self._cmd
        out = copy.copy(c)
        if self._moving:
            ux, uy = dx / dist, dy / dist               # unit direction towards the hand
            v_h = SIGN_H * SPEED * ux                   # sideways speed, mm/s
            v_z = SIGN_V * SPEED * -uy                  # up/down speed, mm/s
            out.theta = c.theta + (v_h / max(c.r, 50.0)) * dt   # sideways mm/s -> base turn
            out.z = _clamp(c.z + v_z * dt, Z_MIN, Z_MAX)
            self.state = "tracking"
            self.debug = (f"hand {dist:.2f} from centre at {angle:.0f}deg -> "
                          f"sideways {v_h:+.0f}mm/s up {v_z:+.0f}mm/s")
        else:
            self.state = "tracking (centred)"
            self.debug = f"hand {dist:.2f} from centre -> in dead zone, holding"
        self._cmd = out
        return out, pinch_to_pincher(sp)
