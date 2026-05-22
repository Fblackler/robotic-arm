"""
robot_arm_pick.py
Visual servoing pick-and-place for a DIY robot arm with a wrist-mounted
webcam (Logitech C920 or similar).

Workflow:
  1. python robot_arm_pick.py --tune        # find HSV thresholds for your object
  2. python robot_arm_pick.py --calibrate   # 4-point homography at scan pose
  3. python robot_arm_pick.py --target      # find where the gripper is in the image
  4. python robot_arm_pick.py --pick        # execute one pick-and-place
  5. python robot_arm_pick.py --pick --loop # keep picking until no object found

Steps 1-3 save settings to robot_config.json. You only redo them if you
move the camera mount, change object color, or change your scan pose.

Assumed Arduino-side serial protocol (you implement on the arm controller):
  Send:  "P x y z\n"      Reply: "OK\n"   move to XYZ in cm, return when settled
  Send:  "POSE name\n"    Reply: "OK\n"   move to named saved pose
  Send:  "G open\n"       Reply: "OK\n"   open gripper
  Send:  "G close\n"      Reply: "OK\n"   close gripper

The script keeps its own copy of "current commanded XYZ" so it does NOT
require the arm to expose a position query. If you want closed-loop on
arm encoders too, swap _current_xyz for a real query.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from robotic_arm_logic import SERVOS
from servo import init_serial

# ===========================================================================
# CONFIGURATION  -- edit these for your hardware
# ===========================================================================

CAMERA_INDEX     = 0
CONFIG_FILE      = 'robot_config.json'

# Named poses — keys into POSES dict below
SCAN_POSE        = "scan"           # high view of the workspace
DROP_POSE        = "drop"           # where to release the object
HOME_POSE        = "home"           # safe parking pose

# Servo angles for each named pose.
# Hobby servos = degrees (0-270). Pincher smart servo = raw (0-4095).
# Tune these values by jogging your physical arm to each position.
POSES = {
    "scan": {"Base": 145, "Arm":  90, "Elbow": 120,
             "Forearm": 175, "Wrist":  80, "Hand": 140},
    "drop": {"Base":  45, "Arm": 100, "Elbow": 120,
             "Forearm": 175, "Wrist":  80, "Hand": 140},
    "home": {"Base": 145, "Arm": 135, "Elbow": 130,
             "Forearm": 175, "Wrist":  80, "Hand": 140},
}

GRIPPER_OPEN  = 1000   # Pincher position when open
GRIPPER_CLOSE = 2042   # Pincher position when closed

# Pick geometry (cm)
HOVER_Z          = 15.0             # XY-centering height above table
GRIP_Z           = 1.0              # height at which gripper closes
SCAN_Z           = 25.0             # height of the camera at scan pose
                                    # (your scan pose should park the arm here)

# Visual servoing parameters
PIXEL_TOLERANCE  = 12               # converged when error magnitude below this
MAX_ITERATIONS   = 8                # safety cap on correction loops
INITIAL_GAIN     = 0.04             # cm per pixel at SCAN_Z (tune empirically)
MAX_STEP_CM      = 3.0              # clip per-iteration correction
DESCENT_FRACTION = 0.30             # fraction of remaining height to descend per iter
DESCEND_THRESHOLD = 4               # only descend when err < N * PIXEL_TOLERANCE

# AXIS CONVENTION: pixel deltas -> world deltas
# Run --pick once; if the arm moves AWAY from the object, flip the sign here.
SIGN_X           = -1               # image +x (right)  -> world * SIGN_X
SIGN_Y           = +1               # image +y (down)   -> world * SIGN_Y

# Detection
MIN_BLOB_AREA    = 500              # ignore blobs smaller than this (pixels)
SETTLE_TIME      = 0.40             # seconds to wait after move before grabbing frame
FRAME_WARMUP     = 3                # discard N stale buffered frames before reading


# ===========================================================================
# CONFIG PERSISTENCE
# ===========================================================================

def load_config():
    p = Path(CONFIG_FILE)
    return json.loads(p.read_text()) if p.exists() else {}


def save_config(cfg):
    Path(CONFIG_FILE).write_text(json.dumps(cfg, indent=2))


# ===========================================================================
# CAMERA HELPERS
# ===========================================================================

def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    # Lock exposure so brightness doesn't shift between frames
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap


def grab_frame(cap, warmup=FRAME_WARMUP):
    """Return a fresh frame, discarding stale ones the OS may have buffered."""
    for _ in range(warmup):
        cap.read()
    ok, frame = cap.read()
    return frame if ok else None


# ===========================================================================
# VISION
# ===========================================================================

def detect_object(frame, hsv_lo, hsv_hi, min_area=MIN_BLOB_AREA):
    """Return (cx, cy, area) for the largest blob matching HSV range, or None."""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lo), np.array(hsv_hi))
    mask = cv2.erode(mask,  None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c    = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None
    M = cv2.moments(c)
    if M['m00'] == 0:
        return None
    return int(M['m10'] / M['m00']), int(M['m01'] / M['m00']), area


# ===========================================================================
# ARM INTERFACE
# ===========================================================================

def _xyz_to_angles(x, y, z):
    """Inverse kinematics — convert target XYZ (cm from arm base) to servo angles.

    TODO: implement this with your arm's actual link lengths.
    Must return a dict: {"Base": deg, "Arm": deg, "Elbow": deg,
                         "Forearm": deg, "Wrist": deg, "Hand": deg}
    Hobby servos use degrees (0-270); Pincher smart servo uses 0-4095.
    """
    raise NotImplementedError(
        "Inverse kinematics not implemented. "
        "Fill in _xyz_to_angles() with your arm's link lengths and IK math."
    )


class Arm:
    def __init__(self):
        init_serial()
        time.sleep(2)
        self._x = self._y = self._z = None

    def move_to_xyz(self, x, y, z):
        angles = _xyz_to_angles(x, y, z)
        for name, angle in angles.items():
            sv = SERVOS.get(name)
            if sv is None:
                continue
            if not (sv.limits[0] <= angle <= sv.limits[1]):
                print(f"  ! {name} angle {angle} out of limits {sv.limits}")
                continue
            sv.write_position(angle)
        self._x, self._y, self._z = x, y, z

    def move_to_pose(self, pose_name):
        angles = POSES.get(pose_name)
        if angles is None:
            print(f"  ! unknown pose '{pose_name}'")
            return
        for name, angle in angles.items():
            sv = SERVOS.get(name)
            if sv is not None:
                sv.write_position(angle)
        self._x = self._y = self._z = None

    def get_xyz(self):
        return self._x, self._y, self._z

    def open_gripper(self):  SERVOS["Pincher"].write_position(GRIPPER_OPEN)
    def close_gripper(self): SERVOS["Pincher"].write_position(GRIPPER_CLOSE)

    def close(self):
        pass  # serial ports are shared; managed by servo.py


# ===========================================================================
# MODE: --tune    (HSV threshold picker)
# ===========================================================================

def mode_tune():
    cfg = load_config()
    cap = open_camera()

    def nothing(_): pass
    cv2.namedWindow('tune')
    init = cfg.get('hsv', {'lo': [0, 100, 100], 'hi': [10, 255, 255]})
    for name, val, mx in [('Hlo', init['lo'][0], 179),
                          ('Hhi', init['hi'][0], 179),
                          ('Slo', init['lo'][1], 255),
                          ('Shi', init['hi'][1], 255),
                          ('Vlo', init['lo'][2], 255),
                          ('Vhi', init['hi'][2], 255)]:
        cv2.createTrackbar(name, 'tune', val, mx, nothing)

    print("Adjust trackbars until 'mask' shows ONLY your object cleanly.")
    print("  s = save and exit    q = quit without saving")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        lo  = [cv2.getTrackbarPos(n, 'tune') for n in ('Hlo', 'Slo', 'Vlo')]
        hi  = [cv2.getTrackbarPos(n, 'tune') for n in ('Hhi', 'Shi', 'Vhi')]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        msk = cv2.inRange(hsv, np.array(lo), np.array(hi))
        cv2.imshow('frame', frame)
        cv2.imshow('mask',  msk)
        cv2.imshow('isolated', cv2.bitwise_and(frame, frame, mask=msk))
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord('s'):
            cfg['hsv'] = {'lo': lo, 'hi': hi}
            save_config(cfg)
            print(f"Saved HSV  lo={lo}  hi={hi}")
            break
    cap.release(); cv2.destroyAllWindows()


# ===========================================================================
# MODE: --calibrate    (4-point homography at scan pose)
# ===========================================================================

# Edit these if you want different calibration positions.
# These are world XY in cm, relative to your arm's base coordinate frame.
CALIB_WORLD_POINTS = np.array(
    [[0, 10], [20, 10], [20, 30], [0, 30]], dtype=np.float32
)

def mode_calibrate():
    arm = Arm()
    print("Moving to scan pose...")
    arm.move_to_pose(SCAN_POSE)
    time.sleep(SETTLE_TIME * 3)

    cap = open_camera()
    frame = grab_frame(cap, warmup=5)
    cap.release()
    arm.close()
    if frame is None:
        print("No frame from camera."); return

    print("\nPlace markers (tape dots, ArUco, anything bright) on the floor at")
    print("these world XY positions in cm (relative to the arm base):")
    for i, p in enumerate(CALIB_WORLD_POINTS, 1):
        print(f"   {i}. ({p[0]:.1f}, {p[1]:.1f})")
    print("\nThen click them IN ORDER in the image window.  q = abort.\n")

    clicks = []
    def on_click(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))
            print(f"   point {len(clicks)} at pixel ({x}, {y})")

    cv2.namedWindow('calibrate')
    cv2.setMouseCallback('calibrate', on_click)
    while len(clicks) < 4:
        disp = frame.copy()
        for i, (x, y) in enumerate(clicks):
            cv2.circle(disp, (x, y), 8, (0, 255, 0), -1)
            cv2.putText(disp, str(i + 1), (x + 10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        nxt = CALIB_WORLD_POINTS[len(clicks)]
        cv2.putText(disp, f"Click point {len(clicks)+1}  world=({nxt[0]:.0f},{nxt[1]:.0f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow('calibrate', disp)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            cv2.destroyAllWindows(); return
    cv2.destroyAllWindows()

    image_pts = np.array(clicks, dtype=np.float32)
    H, _ = cv2.findHomography(image_pts, CALIB_WORLD_POINTS)
    if H is None:
        print("Homography failed."); return

    cfg = load_config()
    cfg['homography'] = H.tolist()
    save_config(cfg)
    print("Saved homography:")
    print(H)


# ===========================================================================
# MODE: --target    (where does the gripper appear in the image?)
# ===========================================================================

def mode_target():
    print("Manually jog the arm so the CLOSED gripper is at grip height (Z≈%g),"
          % GRIP_Z)
    print("with a clearly visible object directly under the fingertips.")
    input("Press Enter when ready... ")
    cap = open_camera()
    frame = grab_frame(cap, warmup=5)
    cap.release()
    if frame is None:
        print("No frame."); return

    clicks = []
    def on_click(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))

    cv2.namedWindow('target')
    cv2.setMouseCallback('target', on_click)
    while not clicks:
        disp = frame.copy()
        cv2.putText(disp, "Click the object under the gripper (q=abort)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow('target', disp)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            cv2.destroyAllWindows(); return
    cv2.destroyAllWindows()

    cfg = load_config()
    cfg['target_pixel'] = list(clicks[0])
    save_config(cfg)
    print(f"Saved target pixel {clicks[0]}")


# ===========================================================================
# THE PICK CYCLE (visual servoing)
# ===========================================================================

def pick_cycle(arm, cap, cfg):
    hsv_lo    = cfg['hsv']['lo']
    hsv_hi    = cfg['hsv']['hi']
    target_px = tuple(cfg['target_pixel'])
    H         = np.array(cfg['homography'])

    print("\n--- Pick cycle ---")
    arm.open_gripper()
    arm.move_to_pose(SCAN_POSE)
    time.sleep(SETTLE_TIME * 2)

    # 1. INITIAL DETECTION using the homography for a coarse XY guess
    frame = grab_frame(cap)
    obj   = detect_object(frame, hsv_lo, hsv_hi)
    if obj is None:
        print("  no object detected at scan pose"); return False
    px, py, area = obj
    print(f"  detected @ pixel ({px},{py}), area={area}")

    pt = np.array([[[px, py]]], dtype=np.float32)
    wx, wy = cv2.perspectiveTransform(pt, H)[0, 0]
    print(f"  initial world estimate: ({wx:.2f}, {wy:.2f}) cm")

    arm.move_to_xyz(float(wx), float(wy), HOVER_Z)
    time.sleep(SETTLE_TIME)

    # 2. ITERATIVE VISUAL CORRECTION
    gain = INITIAL_GAIN
    for it in range(MAX_ITERATIONS):
        frame = grab_frame(cap, warmup=2)
        obj = detect_object(frame, hsv_lo, hsv_hi)
        if obj is None:
            print(f"  [{it}] lost object — aborting"); return False
        px, py, _ = obj
        err_x = px - target_px[0]
        err_y = py - target_px[1]
        err_mag = (err_x * err_x + err_y * err_y) ** 0.5
        print(f"  [{it}] pixel error ({err_x:+d}, {err_y:+d})  mag={err_mag:.1f}")

        if err_mag < PIXEL_TOLERANCE:
            print("  converged"); break

        cx, cy, cz = arm.get_xyz()
        if cx is None:
            print("  no tracked XYZ (named-pose move?) — aborting"); return False

        # pixel error -> world step (with axis-sign convention from config)
        dx = SIGN_X * err_x * gain
        dy = SIGN_Y * err_y * gain
        dx = max(-MAX_STEP_CM, min(MAX_STEP_CM, dx))
        dy = max(-MAX_STEP_CM, min(MAX_STEP_CM, dy))

        # only descend once XY is reasonably close
        if err_mag < DESCEND_THRESHOLD * PIXEL_TOLERANCE:
            new_z = cz - (cz - GRIP_Z) * DESCENT_FRACTION
        else:
            new_z = cz

        arm.move_to_xyz(cx + dx, cy + dy, new_z)
        time.sleep(SETTLE_TIME)
    else:
        print("  did not converge in MAX_ITERATIONS — aborting")
        return False

    # 3. FINAL DESCENT + GRIP
    cx, cy, _ = arm.get_xyz()
    arm.move_to_xyz(cx, cy, GRIP_Z)
    time.sleep(SETTLE_TIME)
    arm.close_gripper()
    time.sleep(0.4)

    # 4. LIFT, DROP, GO HOME
    arm.move_to_xyz(cx, cy, HOVER_Z)
    time.sleep(SETTLE_TIME)
    arm.move_to_pose(DROP_POSE)
    time.sleep(SETTLE_TIME * 2)
    arm.open_gripper()
    time.sleep(0.4)
    arm.move_to_pose(HOME_POSE)
    print("  pick complete")
    return True


def mode_pick(loop=False):
    cfg = load_config()
    for required in ('hsv', 'homography', 'target_pixel'):
        if required not in cfg:
            print(f"Config missing '{required}'. Run --tune / --calibrate / --target first.")
            sys.exit(1)

    arm = Arm()
    cap = open_camera()
    try:
        if loop:
            while pick_cycle(arm, cap, cfg):
                time.sleep(1.0)
            print("No more objects (or failure). Stopping.")
        else:
            pick_cycle(arm, cap, cfg)
    finally:
        cap.release()
        arm.close()


# ===========================================================================
# CLI
# ===========================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g  = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--tune',      action='store_true', help='tune HSV thresholds')
    g.add_argument('--calibrate', action='store_true', help='4-point homography')
    g.add_argument('--target',    action='store_true', help='gripper position in image')
    g.add_argument('--pick',      action='store_true', help='run one pick cycle')
    ap.add_argument('--loop',     action='store_true', help='with --pick: keep picking')
    args = ap.parse_args()

    if   args.tune:      mode_tune()
    elif args.calibrate: mode_calibrate()
    elif args.target:    mode_target()
    elif args.pick:      mode_pick(loop=args.loop)


if __name__ == '__main__':
    main()