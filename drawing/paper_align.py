#!/usr/bin/env python3
"""
paper_align.py
Step 1 of the drawing robot: find a sheet of A4 on the table and line up above it.

The pen and camera point straight down. The arm:
  1. moves to a scan position above the table
  2. finds the paper in the camera
  3. moves sideways / in-out until the paper is centred
  4. rotates the hand until the paper's edges are square with the picture
     (whichever way round is closest, so it never turns more than 45 degrees)
  5. moves down / up until the paper fills the picture by the right amount
  6. holds steady for a second, locks on, and saves the result to paper_pose.json

Every move is at one fixed slow speed, with a dead zone so it doesn't wobble.

    python paper_align.py              # line up with the paper
    python paper_align.py --dry-run    # show what it sees, don't move the arm
    python paper_align.py --no-scan    # start from where the arm is, don't go to the scan pose

Camera view (with the paper outline and what the arm is doing): http://<host>:8090
"""

import argparse
import copy
import json
import math
import os
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "hand_tracking"))   # reuse camera / arm / preview code

import cv2

import hand_tracker
from hand_tracker import ArmClient, Camera, Shared, make_preview_handler, parse_camera
from hand_control import ArmPose
from paper_detector import draw_paper, fills_view, find_paper

# =========================================================
# Settings
# =========================================================

# Where to start looking from. Pen/camera pointing straight down (pitch -90).
# (Checked against the IK: with the pen straight down the arm reaches up to about z=250.)
SCAN_POSE = ArmPose(r=300.0, theta=0.0, z=240.0, pitch=-90.0, roll=0.0)

SPEED_XY   = 25.0     # mm/s, fixed, for centring
SPEED_Z    = 25.0     # mm/s, fixed, for height
SPEED_ROLL = 10.0     # deg/s, fixed, for rotation
SERVER_SPEED_PCT = 20 # how fast the server moves the servos (1-100)

CENTRE_DEADZONE = 0.03   # paper centre within 3% of the picture centre = centred
CENTRE_HYST     = 0.015   # start moving again only past DEADZONE + HYST
TARGET_FILL     = 0.65   # paper's long edge should fill 65% of the picture (width or height, whichever it's along)
FILL_TOL        = 0.04   # +/- 4% counts as right
ANGLE_TOL       = 2.0    # degrees
LOCK_TIME       = 1.0    # s everything must stay lined up before it locks

Z_MIN, Z_MAX = 80.0, 250.0     # never go below / above these heights (mm). Above ~250 is out of reach pointing down
ROLL_LIMIT   = 45.0            # never rotate the hand more than this (it never needs more than 45)

# Directions. Flip one if that axis moves AWAY from lining up.
SIGN_X    = +1   # paper right of centre -> move sideways (base turn)
SIGN_Y    = +1   # paper below centre    -> move in/out (reach)
SIGN_ROLL = +1   # paper rotated clockwise -> which way the hand turns
SIGN_Z    = +1   # paper looks too small -> move down (closer)

RESULT_FILE = Path(HERE) / "paper_pose.json"


# =========================================================
# Control logic (no camera / network, so it can be tested alone)
# =========================================================

class PaperAligner:
    def __init__(self, start_pose, frame_w=640, frame_h=480):
        self.cmd = copy.copy(start_pose)     # last pose asked for
        self.good = copy.copy(start_pose)    # last pose the server accepted
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.state = "looking for paper"
        self.debug = ""
        self.locked = False
        self._centring = False
        self._aligned_since = None
        self._last_t = None

    def confirm(self, pose):
        self.good = copy.copy(pose)

    def reject(self):
        self.cmd = copy.copy(self.good)

    def update(self, paper, now, too_close=False):
        """Returns a pose to send, or None to hold still."""
        dt = 0.0 if self._last_t is None else min(now - self._last_t, 0.2)
        self._last_t = now

        if self.locked:
            return None
        if paper is None and too_close:
            out = copy.copy(self.cmd)
            out.z = min(Z_MAX, out.z + SPEED_Z * dt)
            self.cmd = out
            self.state = "too close"
            self.debug = "paper fills the view - moving up to see its edges"
            self._aligned_since = None
            return out
        if paper is None:
            self.state = "looking for paper"
            self.debug = "no A4 sheet in view - slide it under the camera"
            self._aligned_since = None
            return None

        # ---- errors ----
        ex, ey = paper.cx - 0.5, paper.cy - 0.5
        dist = math.hypot(ex, ey)
        fill = paper.long_px / (self.frame_w if paper.landscape else self.frame_h)
        angle = paper.align_error

        if self._centring and dist < CENTRE_DEADZONE:
            self._centring = False
        elif not self._centring and dist > CENTRE_DEADZONE + CENTRE_HYST:
            self._centring = True

        c = self.cmd
        out = copy.copy(c)
        moves = []

        # 1. centre (fixed speed, in the direction of the paper)
        if self._centring:
            ux, uy = ex / dist, ey / dist
            v_side = SIGN_X * SPEED_XY * ux
            v_reach = SIGN_Y * SPEED_XY * uy
            out.theta = c.theta + (v_side / max(c.r, 50.0)) * dt
            out.r = max(50.0, c.r + v_reach * dt)
            moves.append("centring")

        # 2. rotate (only once roughly centred, so the paper stays in view)
        angle_ok = abs(angle) <= ANGLE_TOL
        if not angle_ok and not paper.cut_off and dist < 0.15:
            step = SIGN_ROLL * SPEED_ROLL * dt * (1 if angle > 0 else -1)
            out.roll = max(-ROLL_LIMIT, min(ROLL_LIMIT, c.roll + step))
            moves.append("rotating")

        # 3. height (only once roughly centred)
        fill_ok = abs(fill - TARGET_FILL) <= FILL_TOL and not paper.cut_off
        if paper.cut_off and dist < 0.15:
            out.z = min(Z_MAX, c.z + SPEED_Z * dt)          # paper too big for the view: go up
            moves.append("moving up (paper cut off)")
        elif not fill_ok and dist < 0.15:
            direction = -1 if fill < TARGET_FILL else +1     # too small -> down
            out.z = max(Z_MIN, min(Z_MAX, c.z + SIGN_Z * direction * SPEED_Z * dt))
            moves.append("moving down" if direction < 0 else "moving up")

        self.cmd = out
        self.debug = (f"centre {dist:.3f}  angle {angle:+.1f}deg  fill {fill:.2f}/{TARGET_FILL:.2f}  "
                      f"| {', '.join(moves) or 'holding'}")

        # ---- lock on when everything is right for LOCK_TIME ----
        aligned = (not self._centring) and angle_ok and fill_ok
        if aligned:
            if self._aligned_since is None:
                self._aligned_since = now
            if now - self._aligned_since >= LOCK_TIME:
                self.locked = True
                self.state = "LOCKED"
                return None
            self.state = "lined up, holding..."
        else:
            self._aligned_since = None
            self.state = "lining up"
        return out if moves else None


def save_result(pose, paper, path=RESULT_FILE):
    data = {
        "arm_pose": {"r": pose.r, "theta_deg": math.degrees(pose.theta), "z": pose.z,
                     "pitch": pose.pitch, "roll": pose.roll, "ik": pose.to_ik(SERVER_SPEED_PCT)},
        "paper": {"mm_per_px": paper.mm_per_px, "angle_in_image_deg": paper.angle,
                  "landscape_in_image": paper.landscape,
                  "centre_in_image": [paper.cx, paper.cy],
                  "long_px": paper.long_px, "short_px": paper.short_px},
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    Path(path).write_text(json.dumps(data, indent=2))
    return data


# =========================================================
# Main
# =========================================================

def main():
    ap = argparse.ArgumentParser(description="Find a sheet of A4 and line the arm up above it.")
    ap.add_argument("--camera", default="0")
    ap.add_argument("--server", default="http://127.0.0.1:8080")
    ap.add_argument("--dry-run", action="store_true", help="don't move the arm")
    ap.add_argument("--no-scan", action="store_true", help="don't move to the scan pose first")
    ap.add_argument("--port", type=int, default=8090)
    args = ap.parse_args()

    hand_tracker.SPEED_PCT = SERVER_SPEED_PCT
    cam = Camera(parse_camera(args.camera))
    arm = ArmClient(args.server, dry_run=args.dry_run)

    if args.no_scan and not args.dry_run:
        start = arm.get_start_pose()
        start.pitch = -90.0
    else:
        start = copy.copy(SCAN_POSE)
    aligner = PaperAligner(start, frame_w=hand_tracker.CAM_WIDTH, frame_h=hand_tracker.CAM_HEIGHT)

    if not args.dry_run:
        print("Moving to the scan pose...")
        arm.send(start, None)
        time.sleep(3)

    shared = Shared()
    if args.port:
        srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_preview_handler(shared))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"Camera view: http://<this-computer's-ip>:{args.port}")
    print("Put a sheet of A4 flat on the table under the camera. Ctrl+C to stop.")

    last_id, last_send, last_preview = -1, 0.0, 0.0
    paper = None
    try:
        while True:
            fid, frame = cam.latest()
            if frame is None or fid == last_id:
                time.sleep(0.005)
                continue
            last_id = fid
            now = time.monotonic()
            aligner.frame_h, aligner.frame_w = frame.shape[:2]

            paper = find_paper(frame)
            for pose, ok in arm.take_results():
                aligner.confirm(pose) if ok else aligner.reject()

            with shared.lock:
                paused = shared.paused
            if not paused:
                pose = aligner.update(paper, now, too_close=(paper is None and fills_view(frame)))
                if pose is not None and now - last_send >= 0.05:
                    arm.send(pose, None)
                    last_send = now

            if aligner.locked and not getattr(aligner, "_saved", False):
                aligner._saved = True
                data = save_result(aligner.good, paper)
                print(f"\n\nLOCKED ON. Saved to {RESULT_FILE.name}:")
                print(f"  arm at r={data['arm_pose']['r']:.0f} mm, base={data['arm_pose']['theta_deg']:.1f} deg, "
                      f"height={data['arm_pose']['z']:.0f} mm, hand roll={data['arm_pose']['roll']:.1f} deg")
                print(f"  scale {data['paper']['mm_per_px']:.3f} mm per pixel")

            state = "PAUSED" if paused else aligner.state
            print(f"\r  {state:<22} {aligner.debug:<80} arm: {arm.status:<20}", end="", flush=True)

            if args.port and now - last_preview >= 1 / 15:
                last_preview = now
                img = frame.copy()
                h, w = img.shape[:2]
                cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)
                cv2.circle(img, (w // 2, h // 2), int(CENTRE_DEADZONE * w), (255, 255, 255), 1)
                tw = int(TARGET_FILL * w)
                cv2.rectangle(img, ((w - tw) // 2, h // 2 - 3), ((w + tw) // 2, h // 2 + 3), (120, 120, 120), 1)
                draw_paper(img, paper, locked=aligner.locked)
                cv2.rectangle(img, (0, 0), (w, 30), (0, 0, 0), -1)
                col = (80, 220, 80) if aligner.locked else (255, 255, 255)
                cv2.putText(img, state.upper(), (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
                cv2.rectangle(img, (0, h - 26), (w, h), (0, 0, 0), -1)
                cv2.putText(img, aligner.debug, (8, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 220, 255), 1)
                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    with shared.lock:
                        shared.jpeg = jpg.tobytes()
                        shared.jpeg_id += 1
                        shared.status.update(state=state, arm=arm.status, fps=0)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopped. The arm stays where it is.")
        cam.close()


if __name__ == "__main__":
    main()
