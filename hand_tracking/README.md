# Hand tracking

Moves the arm to keep a hand centred in the view of a camera mounted on the arm. It sends targets to the server's `/ik` endpoint.

## Run

```bash
pip install -r requirements.txt       # Python 3.9–3.12
python hand_tracker.py                # --dry-run to test without moving the arm
```

The hand model downloads to `cache/` on first run.

- **Camera view:** `http://<host>:8090`, with a **Pause arm** button.
- **Options:**
  - `--camera N` to pick a camera
  - `--server http://host:8080` to point at a different server
  - `--port 0` to turn off the preview
  - `--show` to open a local window

## Controls

| Hand | Arm |
|---|---|
| Open hand, held still for 0.5 s | Starts following |
| Inside the centre circle | Holds still (dead zone) |
| Outside the circle | Moves towards the hand at a fixed speed |
| Fist, or no hand | Stops |
| Thumb-index pinch | Closes the gripper |

## Settings

Settings are at the top of `hand_control.py`:

| Setting | Purpose |
|---|---|
| `MODE` | `simple` (default), `follow`, `absolute` or `relative` |
| `SIMPLE_SPEED` | Movement speed in mm/s |
| `SIMPLE_DEADZONE` | Radius of the centre dead zone |
| `SIMPLE_SIGN_H`, `SIMPLE_SIGN_V` | Flip left/right or up/down |
| `PINCH_CLOSED`, `PINCH_OPEN` | Gripper pinch thresholds |

`MIRROR` in `hand_tracker.py` flips the image for a desk-mounted camera facing the user.
