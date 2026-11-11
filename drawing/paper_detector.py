"""
paper_detector.py
Finds a sheet of white paper (A4) in a camera image, seen from above.

Returns where it is in the picture, how big it looks, and how it's rotated.
No arm code in here, so it can be tested on its own with photos.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

A4_LONG_MM, A4_SHORT_MM = 297.0, 210.0

MIN_AREA_FRAC   = 0.03   # ignore white shapes smaller than 3% of the picture
MIN_RECTANGULAR = 0.85   # shape must fill 85% of its bounding rectangle
ASPECT_TOL      = 0.25   # allowed difference from the A4 ratio (1.414)
BORDER_PX       = 4      # paper this close to the picture edge counts as "cut off"
MAX_AREA_FRAC   = 0.92   # bigger than this = it's the whole table, not paper
MIN_CONTRAST    = 35     # paper must be this much brighter than the table (0-255)


@dataclass
class Paper:
    cx: float            # centre, 0..1 across the picture
    cy: float            # centre, 0..1 down the picture
    long_px: float       # length of the long edge in pixels
    short_px: float      # length of the short edge in pixels
    angle: float         # degrees: long edge vs horizontal, -90..90 (0 = long edge left-right)
    align_error: float   # degrees to turn to square it up with the nearest picture edge, -45..45
    landscape: bool      # True if the long edge is closer to horizontal than vertical
    corners: np.ndarray  # 4x2 pixel corners
    cut_off: bool        # True if the paper runs off the edge of the picture
    mm_per_px: float     # scale at the paper, from its known size


def _long_edge_angle(box):
    """Angle of the longer edge of a 4-point box, folded into -90..90."""
    e1 = box[1] - box[0]
    e2 = box[2] - box[1]
    e = e1 if np.hypot(*e1) >= np.hypot(*e2) else e2
    a = math.degrees(math.atan2(e[1], e[0]))
    while a > 90:
        a -= 180
    while a <= -90:
        a += 180
    return a


def find_paper(frame):
    """Return a Paper, or None if no A4-shaped sheet is visible."""
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    # Paper is the bright thing: Otsu picks the threshold automatically
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_FRAC * w * h or area > MAX_AREA_FRAC * w * h:
            continue
        # must be clearly brighter than its surroundings
        inside = np.zeros((h, w), np.uint8)
        cv2.drawContours(inside, [c], -1, 255, -1)
        if cv2.mean(gray, mask=inside)[0] - cv2.mean(gray, mask=255 - inside)[0] < MIN_CONTRAST:
            continue
        rect = cv2.minAreaRect(c)
        (rw, rh) = rect[1]
        if rw < 1 or rh < 1:
            continue
        if area / (rw * rh) < MIN_RECTANGULAR:
            continue
        box = cv2.boxPoints(rect)
        x0, y0 = box.min(axis=0)
        x1, y1 = box.max(axis=0)
        cut = x0 <= BORDER_PX or y0 <= BORDER_PX or x1 >= w - BORDER_PX or y1 >= h - BORDER_PX
        aspect = max(rw, rh) / min(rw, rh)
        if not cut and abs(aspect - A4_LONG_MM / A4_SHORT_MM) > ASPECT_TOL:
            continue            # a whole rectangle, but not A4 shaped
        if best is None or area > best[0]:
            best = (area, rect, box, cut)

    if best is None:
        return None
    _, rect, box, cut = best
    (cx, cy), (rw, rh), _ = rect
    long_px, short_px = max(rw, rh), min(rw, rh)
    angle = _long_edge_angle(box)
    landscape = abs(angle) <= 45
    align_error = angle if landscape else (angle - 90 if angle > 0 else angle + 90)
    return Paper(
        cx=cx / w, cy=cy / h,
        long_px=long_px, short_px=short_px,
        angle=angle,
        align_error=align_error,
        landscape=landscape,
        corners=box,
        cut_off=cut,
        mm_per_px=A4_LONG_MM / long_px,
    )


def fills_view(frame):
    """True if the picture is almost all white paper (camera too close to see the edges)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(np.mean(gray > 170)) > 0.85


def draw_paper(img, paper, locked=False):
    """Outline the detected paper on the preview image."""
    if paper is None:
        return
    col = (80, 220, 80) if locked else ((0, 165, 255) if paper.cut_off else (255, 200, 0))
    pts = paper.corners.astype(int).reshape(-1, 1, 2)
    cv2.polylines(img, [pts], True, col, 2)
    h, w = img.shape[:2]
    c = (int(paper.cx * w), int(paper.cy * h))
    cv2.circle(img, c, 6, col, -1)
    # long-edge direction arrow
    a = math.radians(paper.angle)
    tip = (int(c[0] + 60 * math.cos(a)), int(c[1] + 60 * math.sin(a)))
    cv2.arrowedLine(img, c, tip, col, 2, tipLength=0.25)
