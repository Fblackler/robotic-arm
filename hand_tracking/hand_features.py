"""
hand_features.py
Turns the 21 MediaPipe hand landmarks into a few numbers the arm can use,
plus the smoothing filter that stops the arm twitching.

MediaPipe landmark indices used here:
    0 wrist
    4 thumb tip
    5 index knuckle   6 index middle joint   8 index tip
    9 middle knuckle 10 middle middle joint 12 middle tip
   13 ring knuckle   14 ring middle joint   16 ring tip
   17 pinky knuckle  18 pinky middle joint  20 pinky tip
"""

import math
from dataclasses import dataclass

WRIST = 0
THUMB_TIP = 4
INDEX_MCP, INDEX_PIP, INDEX_TIP = 5, 6, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_TIP = 9, 10, 12
RING_MCP, RING_PIP, RING_TIP = 13, 14, 16
PINKY_MCP, PINKY_PIP, PINKY_TIP = 17, 18, 20

# Bones to draw on the preview image
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]


@dataclass
class HandFeatures:
    x: float           # palm centre across the image, 0 (left) .. 1 (right)
    y: float           # palm centre down the image,   0 (top)  .. 1 (bottom)
    size: float        # palm size as a fraction of image height (bigger = closer)
    roll: float        # degrees the hand is tilted in the image, 0 = fingers up
    pinch: float       # thumb-to-index gap / palm size (~0.1 touching, ~1 open)
    fist: bool         # at least 3 fingers curled
    handedness: str    # "Left" / "Right" as MediaPipe sees it


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def extract_features(landmarks, image_w, image_h, handedness="?"):
    """landmarks: list of 21 objects with .x/.y in 0..1 (MediaPipe NormalizedLandmark)."""
    aspect = image_w / float(image_h)
    # Scale x by the aspect ratio so distances are the same in both directions
    p = [(lm.x * aspect, lm.y) for lm in landmarks]

    palm = [p[WRIST], p[INDEX_MCP], p[MIDDLE_MCP], p[PINKY_MCP]]
    cx = sum(q[0] for q in palm) / 4.0
    cy = sum(q[1] for q in palm) / 4.0

    # Average of the palm triangle's sides. Less affected by the hand
    # turning than a single wrist-to-knuckle distance.
    size = (_dist(p[WRIST], p[INDEX_MCP]) +
            _dist(p[WRIST], p[PINKY_MCP]) +
            _dist(p[INDEX_MCP], p[PINKY_MCP])) / 3.0
    size = max(size, 1e-4)

    dx = p[MIDDLE_MCP][0] - p[WRIST][0]
    dy = p[MIDDLE_MCP][1] - p[WRIST][1]
    roll = math.degrees(math.atan2(dx, -dy))

    pinch = _dist(p[THUMB_TIP], p[INDEX_TIP]) / size

    curled = 0
    for tip, pip in ((INDEX_TIP, INDEX_PIP), (MIDDLE_TIP, MIDDLE_PIP),
                     (RING_TIP, RING_PIP), (PINKY_TIP, PINKY_PIP)):
        if _dist(p[WRIST], p[tip]) < _dist(p[WRIST], p[pip]):
            curled += 1

    return HandFeatures(
        x=cx / aspect, y=cy, size=size, roll=roll,
        pinch=pinch, fist=curled >= 3, handedness=handedness,
    )


class OneEuroFilter:
    """Smoothing filter: heavy smoothing when the hand is still (kills jitter),
    light smoothing when it moves fast (keeps lag low).

    min_cutoff: lower = smoother when still.  beta: higher = less lag when moving.
    """

    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.reset()

    def reset(self):
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(dt, cutoff):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.x_prev is None:
            self.x_prev, self.t_prev = x, t
            return x
        dt = t - self.t_prev
        if dt <= 0:
            return self.x_prev
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(dt, self.d_cutoff)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(dt, cutoff)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat
