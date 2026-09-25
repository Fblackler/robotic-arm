# Hand tracking

Move your hand in front of the webcam and the arm follows it.

`hand_tracker.py` runs on the arm server next to `full_servo_hud.py`. It sends targets to `/ik` and `/pincher`, the same way the Xbox controller script does, so the server doesn't need any changes.

## Files

| File | What it does |
|---|---|
| `hand_tracker.py` | Main script: camera, hand detection, sends targets to the arm, browser preview |
| `hand_control.py` | Turns hand movement into arm moves. **All the tuning is at the top of this file.** |
| `hand_features.py` | Hand landmark maths (palm position, size, pinch, fist) and smoothing |
| `requirements-hand.txt` | Python packages |

The hand model (`cache/hand_landmarker.task`, ~8 MB) downloads automatically the first time you run it.

## Setup (on the Ubuntu server)

Newer Ubuntu versions don't let `pip` install into the system Python, so use a virtual environment:

```bash
cd robotic-arm-vision
sudo apt install python3-venv
python3 -m venv venv
source venv/bin/activate          # do this in every new terminal before running anything
pip install flask pyserial numpy requests evdev
pip install -r requirements-hand.txt
sudo usermod -aG dialout,video,input $USER   # USB serial, camera, Xbox pad. Log out and back in after.
ls /dev/video*                    # the C920 should show up, usually /dev/video0
```

If `import cv2` complains about `libGL.so.1`, run `sudo apt install libgl1`.

## Running

1. Start the arm server as usual: `python full_servo_hud.py`
2. Try it without moving the arm first:
   ```bash
   python hand_tracker.py --dry-run
   ```
3. Open **http://&lt;server-ip&gt;:8090** on your laptop or phone to see the camera view, with the hand skeleton drawn on it.
4. When it looks right, run it for real:
   ```bash
   python hand_tracker.py
   ```

Options: `--camera 1` or `--camera /dev/video2` for a different camera, `--server http://host:8080`, `--port 0` to turn off the preview, `--show` for a local window if the server has a screen.

## Controls

| Hand | Arm |
|---|---|
| Open hand, hold still ~0.5 s | Arm starts following (it does **not** jump to your hand) |
| Left / right | Base turns |
| Up / down | Arm up / down |
| Towards / away from camera | Reach further / closer |
| Pinch thumb + index | Close pincher (partly pinched = partly closed) |
| Fist, or take your hand away | Arm holds still |

It works like a mouse: movement is **relative**. If you run out of room, make a fist, move your hand back to the middle, and open it again.

The preview page has a **Pause arm** button. Ctrl+C stops the tracker, and the arm stays where it is.

## Tuning (`hand_control.py`)

- **An axis goes the wrong way:** flip `SIGN_THETA`, `SIGN_Z` or `SIGN_R`. Which way is "right" depends on where the camera sits relative to the arm.
- **Too sensitive or too slow:** change `THETA_GAIN`, `Z_GAIN`, `R_GAIN`. Top speeds are `R_RATE`, `THETA_RATE`, `Z_RATE`.
- **Jittery:** lower `min_cutoff` on the filters in `HandArmController.__init__`. **Laggy:** raise `beta`.
- **Pincher closes too early or late:** change `PINCH_CLOSED` / `PINCH_OPEN`. The preview line between thumb and index turns red when the pinch counts as closed.
- **Hand roll:** set `ENABLE_ROLL = True` to rotate the gripper by tilting your hand.

## Tips

- Put the camera on the desk facing you, about 50–80 cm away, with a plain background and good light.
- Reaching (towards/away from the camera) is the least precise axis, because it's worked out from how big your palm looks. Keep your palm facing the camera.
- If the arm stops at the edge of its reach, the tracker keeps the last position that worked. The preview shows `out_of_reach [joint]`.
