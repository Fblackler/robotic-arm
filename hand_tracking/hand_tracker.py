#!/usr/bin/env python3
"""
hand_tracker.py
Drive the robot arm by moving your hand in front of a webcam (Logitech C920 etc).

Runs on the arm server next to full_servo_hud.py and talks to it the same way
servo_controller.py does (POST /ik and /pincher), so the server needs no changes.

    python hand_tracker.py               # track your hand and move the arm
    python hand_tracker.py --dry-run     # track only, arm doesn't move (try this first)
    python hand_tracker.py --camera 1    # different camera (index or /dev/videoN)
    python hand_tracker.py --show        # also open a window (needs a screen)

Watch what the camera sees from any device on your network:
    http://<server-ip>:8090
The page has Pause / Resume buttons.

Controls (settings are at the top of hand_control.py):
    open hand, hold still ~0.5 s     ->  arm starts following
    hand inside the centre circle    ->  arm holds still
    hand outside the circle          ->  arm moves towards it at a fixed slow speed
    pinch thumb + index              ->  close gripper
    make a fist or remove hand       ->  arm stops
"""

import argparse
import copy
import os
import platform
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import requests

import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

from hand_features import HAND_CONNECTIONS, extract_features
import hand_control
from hand_control import ArmPose, HandArmController

# =========================================================
# Config
# =========================================================

SERVER        = "http://localhost:8080"   # full_servo_hud.py
SPEED_PCT     = 30                        # servo speed on the server, 1-100 (was 70)
SEND_HZ       = 20                        # how often to send arm targets
PREVIEW_PORT  = 8090
PREVIEW_FPS   = 15

CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640, 480, 30
MIRROR = False       # True = flip the picture like a selfie (for a camera facing you on the desk).
                     # False for a camera mounted on the arm.   # plenty for hand tracking, keeps CPU low

MODEL_PATH = Path(__file__).resolve().parent / "cache" / "hand_landmarker.task"
MODEL_URL  = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
              "hand_landmarker/float16/latest/hand_landmarker.task")


# =========================================================
# Model
# =========================================================

def ensure_model():
    if MODEL_PATH.exists() and MODEL_PATH.stat().st_size > 1_000_000:
        return str(MODEL_PATH)
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading hand model to {MODEL_PATH} ...")
    tmp = MODEL_PATH.with_suffix(".part")
    urllib.request.urlretrieve(MODEL_URL, tmp)
    tmp.replace(MODEL_PATH)
    return str(MODEL_PATH)


def make_landmarker():
    opts = vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=ensure_model()),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.6,
        min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.5,
    )
    return vision.HandLandmarker.create_from_options(opts)


# =========================================================
# Camera (background thread, always hands out the newest frame)
# =========================================================

class Camera:
    def __init__(self, source):
        self.source = source
        self.frame = None
        self.frame_id = 0
        self.ok = False
        self._lock = threading.Lock()
        self._stop = False
        self._cap = None
        threading.Thread(target=self._run, daemon=True).start()

    def _open(self):
        src = self.source
        if isinstance(src, str) and os.path.isfile(src):
            return cv2.VideoCapture(src)                     # recorded video, for testing
        if platform.system() == "Linux":
            cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        elif platform.system() == "Windows" and isinstance(src, int):
            cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(src)
        # MJPG lets the C920 do 30 fps over USB
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _run(self):
        while not self._stop:
            if self._cap is None or not self._cap.isOpened():
                self._cap = self._open()
                if not self._cap.isOpened():
                    self.ok = False
                    print(f"\n[camera] can't open {self.source!r}, retrying in 2 s")
                    time.sleep(2)
                    continue
            got, frame = self._cap.read()
            if not got:
                self.ok = False
                self._cap.release()
                self._cap = None
                time.sleep(0.5)
                continue
            with self._lock:
                self.frame = frame
                self.frame_id += 1
                self.ok = True
            if isinstance(self.source, str) and os.path.isfile(self.source):
                time.sleep(1.0 / CAM_FPS)                    # play test videos at real speed

    def latest(self):
        with self._lock:
            return self.frame_id, self.frame

    def close(self):
        self._stop = True
        if self._cap is not None:
            self._cap.release()


# =========================================================
# Arm client (background thread, always sends the newest target)
# =========================================================

class ArmClient:
    """Sends the latest pose to /ik without blocking the camera loop.
    Old targets are dropped if the server is slower than we are."""

    def __init__(self, server, dry_run=False):
        self.server = server.rstrip("/")
        self.dry_run = dry_run
        self.session = requests.Session()
        self.status = "dry run" if dry_run else "waiting"
        self.results = []                # [(pose, in_reach)] for the main loop
        self._pending_pose = None
        self._pending_pincher = None
        self._last_pincher = None
        self._cv = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def get_start_pose(self):
        """Start from wherever the arm was last sent (by the HUD or the Xbox pad)."""
        try:
            t = self.session.get(self.server + "/state", timeout=1).json().get("target")
            if t:
                return ArmPose.from_ik(t)
        except Exception:
            pass
        return ArmPose()

    def send(self, pose, pincher):
        with self._cv:
            if pose is not None:
                self._pending_pose = copy.copy(pose)
            if pincher is not None and (self._last_pincher is None or
                                        abs(pincher - self._last_pincher) >= 10):
                self._pending_pincher = int(pincher)
            self._cv.notify()

    def take_results(self):
        with self._cv:
            r, self.results = self.results, []
        return r

    def _run(self):
        while True:
            with self._cv:
                while self._pending_pose is None and self._pending_pincher is None:
                    self._cv.wait()
                pose, self._pending_pose = self._pending_pose, None
                pincher, self._pending_pincher = self._pending_pincher, None

            if self.dry_run:
                if pose is not None:
                    with self._cv:
                        self.results.append((pose, True))
                if pincher is not None:
                    self._last_pincher = pincher
                continue

            if pincher is not None:
                try:
                    self.session.post(self.server + "/pincher",
                                      json={"position": pincher}, timeout=0.2)
                    self._last_pincher = pincher
                except Exception:
                    pass

            if pose is not None:
                ok = False
                try:
                    res = self.session.post(self.server + "/ik",
                                            json=pose.to_ik(SPEED_PCT), timeout=0.3).json()
                    st = res.get("status", "?")
                    ok = st == "in_reach"
                    oob = res.get("out_of_range", [])
                    self.status = st + (f" [{', '.join(oob)}]" if oob else "")
                    if st == "error":
                        self.status = "IK error: " + str(res.get("message", ""))[:40]
                except Exception:
                    self.status = "no response from server"
                with self._cv:
                    self.results.append((pose, ok))


# =========================================================
# Browser preview (MJPEG) so you can see it on a headless server
# =========================================================

PREVIEW_HTML = b"""<!DOCTYPE html>
<html><head><title>Hand tracker</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
 body{background:#111;color:#ddd;font-family:Arial,sans-serif;margin:0;padding:16px;text-align:center}
 img{max-width:100%;border:1px solid #333;border-radius:6px}
 button{padding:12px 28px;margin:10px 6px;font-size:16px;border-radius:6px;border:1px solid #39f;
        background:#1a3a5a;color:#7cf;cursor:pointer}
 #pause{border-color:#f55;background:#4a1a1a;color:#f99}
 #st{font-family:monospace;margin-top:6px;color:#aaa}
</style></head><body>
<h2>Hand tracker</h2>
<img src="/stream"><br>
<button id="pause" onclick="fetch('/pause',{method:'POST'})">Pause arm</button>
<button onclick="fetch('/resume',{method:'POST'})">Resume</button>
<div id="st">...</div>
<script>
setInterval(()=>fetch('/status').then(r=>r.json()).then(d=>{
 document.getElementById('st').textContent =
  (d.paused?'PAUSED  |  ':'') + d.state + '  |  arm: ' + d.arm + '  |  ' + d.fps + ' fps';
}).catch(()=>{}),300);
</script></body></html>"""


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.jpeg_id = 0
        self.paused = False
        self.status = {"state": "starting", "arm": "", "fps": 0, "paused": False}


def make_preview_handler(shared):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj):
            import json
            body = json.dumps(obj).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(PREVIEW_HTML)
            elif self.path == "/status":
                with shared.lock:
                    st = dict(shared.status, paused=shared.paused)
                self._json(st)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                last = -1
                try:
                    while True:
                        with shared.lock:
                            jid, jpg = shared.jpeg_id, shared.jpeg
                        if jpg is None or jid == last:
                            time.sleep(0.02)
                            continue
                        last = jid
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(jpg)).encode() +
                                         b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path in ("/pause", "/resume"):
                with shared.lock:
                    shared.paused = self.path == "/pause"
                self._json({"paused": shared.paused})
            else:
                self.send_error(404)

    return Handler


# =========================================================
# Drawing
# =========================================================

def draw_overlay(img, landmarks, feat, state, arm_status, paused, fps, debug=""):
    h, w = img.shape[:2]
    if landmarks:
        pts = [(int(l.x * w), int(l.y * h)) for l in landmarks]
        colour = (0, 200, 255) if state.startswith("tracking") else (160, 160, 160)
        for a, b in HAND_CONNECTIONS:
            cv2.line(img, pts[a], pts[b], colour, 2)
        for p in pts:
            cv2.circle(img, p, 3, (255, 255, 255), -1)
        if feat is not None:
            c = (int(feat.x * w), int(feat.y * h))
            cv2.circle(img, c, 8, (0, 255, 0), 2)
            closed = feat.pinch < 0.45
            cv2.line(img, pts[4], pts[8], (0, 0, 255) if closed else (0, 255, 0), 2)

    # Centre dead zone: the arm holds still while your palm is inside this circle
    cx, cy = w // 2, h // 2
    cv2.circle(img, (cx, cy), int(hand_control.DEADZONE * h), (255, 200, 0), 1)
    cv2.line(img, (cx - 10, cy), (cx + 10, cy), (255, 200, 0), 1)
    cv2.line(img, (cx, cy - 10), (cx, cy + 10), (255, 200, 0), 1)

    if paused:
        label, col = "PAUSED", (60, 60, 255)
    elif state.startswith("tracking"):
        label, col = state.upper(), (80, 220, 80)
    else:
        label, col = state.upper(), (200, 200, 200)
    cv2.rectangle(img, (0, 0), (w, 34), (0, 0, 0), -1)
    cv2.putText(img, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
    cv2.rectangle(img, (0, h - 48), (w, h), (0, 0, 0), -1)
    cv2.putText(img, f"arm: {arm_status}", (10, h - 29),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    if debug:
        cv2.putText(img, debug, (10, h - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 220, 255), 1)
    cv2.putText(img, f"{fps:4.1f} fps", (w - 90, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)


# =========================================================
# Main
# =========================================================

def parse_camera(s):
    return int(s) if s.isdigit() else s


def main():
    ap = argparse.ArgumentParser(description="Control the robot arm with your hand.")
    ap.add_argument("--camera", default="0", help="camera index or path, e.g. 0 or /dev/video0")
    ap.add_argument("--server", default="http://127.0.0.1:8080", help="full_servo_hud.py address")
    ap.add_argument("--dry-run", action="store_true", help="track only, don't move the arm")
    ap.add_argument("--show", action="store_true", help="open a local preview window")
    ap.add_argument("--port", type=int, default=PREVIEW_PORT, help="browser preview port (0 = off)")
    args = ap.parse_args()

    landmarker = make_landmarker()
    cam = Camera(parse_camera(args.camera))
    arm = ArmClient(args.server, dry_run=args.dry_run)

    start_pose = ArmPose() if args.dry_run else arm.get_start_pose()
    ctrl = HandArmController(start_pose)

    shared = Shared()
    if args.port:
        srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_preview_handler(shared))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"Preview: http://<this-server's-ip>:{args.port}")

    print("Show an open hand to the camera and hold still to start. Fist = let go. Ctrl+C to quit.")
    if args.dry_run:
        print("DRY RUN: the arm will not move.")

    last_id = -1
    last_send = 0.0
    last_preview = 0.0
    fps, fps_t, fps_n = 0.0, time.time(), 0
    t0 = time.monotonic()
    ts_last = -1

    try:
        while True:
            fid, frame = cam.latest()
            if frame is None or fid == last_id:
                if not cam.ok:
                    with shared.lock:
                        shared.status.update(state="no camera", arm=arm.status, fps=0)
                time.sleep(0.003)
                continue
            last_id = fid

            if MIRROR:
                frame = cv2.flip(frame, 1)             # mirror, like a selfie
            h, w = frame.shape[:2]
            ctrl.aspect = w / h
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            ts = int((time.monotonic() - t0) * 1000)
            ts = max(ts, ts_last + 1)                  # MediaPipe needs increasing timestamps
            ts_last = ts
            result = landmarker.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

            landmarks, feat = None, None
            if result.hand_landmarks:
                landmarks = result.hand_landmarks[0]
                hand = result.handedness[0][0].category_name if result.handedness else "?"
                feat = extract_features(landmarks, w, h, hand)

            now = time.monotonic()
            with shared.lock:
                paused = shared.paused

            # Feed back what the server said about earlier targets
            for pose, ok in arm.take_results():
                ctrl.confirm(pose) if ok else ctrl.reject()

            if paused:
                ctrl.release()
                ctrl.update(None, now)
                ctrl.state = "paused"
            else:
                pose, pincher = ctrl.update(feat, now)
                if (pose is not None or pincher is not None) and now - last_send >= 1.0 / SEND_HZ:
                    arm.send(pose, pincher)
                    last_send = now

            fps_n += 1
            if time.time() - fps_t >= 1.0:
                fps = fps_n / (time.time() - fps_t)
                fps_t, fps_n = time.time(), 0

            # Status line
            p = ctrl.pose
            print(f"\r  {ctrl.state:<26} r={p.r:5.0f} th={p.theta*57.2958:6.1f} z={p.z:5.0f}"
                  f"  | arm: {arm.status:<28} {fps:4.1f}fps", end="", flush=True)

            want_preview = args.port and now - last_preview >= 1.0 / PREVIEW_FPS
            if want_preview or args.show:
                draw_overlay(frame, landmarks, feat, ctrl.state, arm.status, paused, fps,
                             ctrl.debug if feat is not None else "")
            if want_preview:
                last_preview = now
                ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    with shared.lock:
                        shared.jpeg = jpg.tobytes()
                        shared.jpeg_id += 1
                        shared.status.update(state=ctrl.state, arm=arm.status, fps=round(fps, 1))
            if args.show:
                cv2.imshow("hand tracker", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        print("\nExiting. The arm stays where it is.")
        cam.close()
        landmarker.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
