# Drawing robot

The arm draws on a sheet of A4 lying flat on the table, with a pen pointing straight down.

## Step 1: find and line up with the paper (`paper_align.py`)

The camera finds the paper. The arm then:
- moves until the paper is centred in the view
- rotates the hand until the paper is square to the camera
- moves up or down until the paper is the right size in the picture

After holding steady for a second, it locks on and saves the arm position, paper angle and scale (mm per pixel) to `paper_pose.json`.

```bash
python paper_align.py --dry-run    # just watch the detection
python paper_align.py              # line up for real
```

It uses the camera, server connection and preview code from `../hand_tracking`, so it needs the same packages. Camera view: `http://<host>:8090`.

### Settings (top of `paper_align.py`)

| Setting | Purpose |
|---|---|
| `SCAN_POSE` | Where it starts looking from (pen straight down) |
| `SPEED_XY`, `SPEED_Z`, `SPEED_ROLL` | Fixed movement speeds |
| `CENTRE_DEADZONE`, `ANGLE_TOL`, `FILL_TOL` | How close counts as lined up |
| `TARGET_FILL` | How much of the picture the paper should fill (sets the height) |
| `SIGN_X`, `SIGN_Y`, `SIGN_ROLL`, `SIGN_Z` | Flip if an axis moves away from lining up |

## Next

Step 2: work out the pen tip's position relative to the camera, then draw shapes and images on the paper.
