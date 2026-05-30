import math
import time
import threading
import requests
from evdev import InputDevice, ecodes, list_devices

# =========================================================
# Config
# =========================================================

SERVER         = "http://localhost:8080/ik"
SERVER_PINCHER = "http://localhost:8080/pincher"
SPEED_PCT      = 70
DEADZONE       = 0.12
LOOP_HZ        = 20
DT             = 1.0 / LOOP_HZ

R_RATE       = 200.0   # mm/s
THETA_RATE   = 0.4     # rad/s  — kept slow for fine base rotation
Z_RATE       = 150.0   # mm/s
PITCH_RATE   = 50.0    # deg/s
ROLL_RATE    = 60.0    # deg/s
YAW_RATE     = 60.0    # deg/s
PINCHER_RATE = 150.0   # position units/s  (range 1000–2200)

PINCHER_MIN = 1000.0
PINCHER_MAX = 2200.0

# Xbox One axis/button codes (Linux xpad driver)
AX_LX  = ecodes.ABS_X    # Left stick X
AX_LY  = ecodes.ABS_Y    # Left stick Y
AX_RX  = ecodes.ABS_RX   # Right stick X
AX_RY  = ecodes.ABS_RY   # Right stick Y
AX_LT  = ecodes.ABS_Z    # Left trigger  (0 = released → open)
AX_RT  = ecodes.ABS_RZ   # Right trigger (0 = released → close)

BTN_A  = ecodes.BTN_SOUTH  # A  — Z down
BTN_Y  = ecodes.BTN_NORTH  # Y  — Z up
BTN_LB = ecodes.BTN_TL     # LB — yaw left
BTN_RB = ecodes.BTN_TR     # RB — yaw right


# =========================================================
# Device discovery
# =========================================================

def find_xbox():
    for path in list_devices():
        try:
            dev = InputDevice(path)
            if any(k in dev.name.lower() for k in ("xbox", "x-box", "microsoft")):
                return dev
        except Exception:
            continue
    return None


# =========================================================
# Controller state (updated by background reader thread)
# =========================================================

_axes = {}
_btns = {}
_lock = threading.Lock()


def _reader(dev):
    for event in dev.read_loop():
        with _lock:
            if event.type == ecodes.EV_ABS:
                _axes[event.code] = event.value
            elif event.type == ecodes.EV_KEY:
                _btns[event.code] = event.value


def axis(code, absinfo):
    """Normalised: sticks → -1..1, triggers → 0..1."""
    with _lock:
        raw = _axes.get(code, absinfo[code].value)
    info = absinfo[code]
    if info.min == 0:
        return raw / max(info.max, 1)
    mid  = (info.min + info.max) / 2.0
    half = (info.max - info.min) / 2.0
    return (raw - mid) / half if half else 0.0


def btn(code):
    with _lock:
        return _btns.get(code, 0)


def dead(v):
    if abs(v) < DEADZONE:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - DEADZONE) / (1.0 - DEADZONE)


# =========================================================
# Requests
# =========================================================

def post_ik(x, y, z, pitch, yaw, roll):
    try:
        resp = requests.post(SERVER, json={
            "X": round(x, 1), "Y": round(y, 1), "Z": round(z, 1),
            "Pitch": round(pitch, 2), "Yaw": round(yaw, 2),
            "Roll":  round(roll,  2), "Speed": SPEED_PCT,
        }, timeout=0.15)
        return resp.json()
    except Exception:
        return None


def post_pincher(pos):
    try:
        requests.post(SERVER_PINCHER, json={"position": int(pos)}, timeout=0.1)
    except Exception:
        pass


# =========================================================
# Main
# =========================================================

def main():
    dev = find_xbox()
    if dev is None:
        print("No Xbox controller found.")
        print("Available devices:")
        for path in list_devices():
            try:
                print(f"  {path}  {InputDevice(path).name}")
            except Exception:
                pass
        return

    print(f"Connected: {dev.name}  ({dev.path})")
    print()
    print("  Left stick up / down       →  arm further / closer")
    print("  Left stick left / right    →  rotate around base (slow)")
    print("  A / Y                      →  Z down / Z up")
    print("  Right stick up / down      →  pitch")
    print("  Right stick left / right   →  hand roll")
    print("  LB / RB                    →  yaw")
    print("  Left trigger               →  pincher open")
    print("  Right trigger              →  pincher close / grip")
    print()

    absinfo = {}
    caps = dev.capabilities()
    with _lock:
        for code, info in caps.get(ecodes.EV_ABS, []):
            absinfo[code] = info
            _axes[code] = info.value

    t = threading.Thread(target=_reader, args=(dev,), daemon=True)
    t.start()

    # Control state
    r           = 300.0
    theta       = 0.0
    z           = 350.0
    pitch       = -20.0
    yaw_offset  =   0.0
    roll        =   0.0
    pincher_pos = 1500.0

    while True:
        t0 = time.time()

        lx = dead(axis(AX_LX, absinfo))
        ly = dead(axis(AX_LY, absinfo))
        rx = dead(axis(AX_RX, absinfo))
        ry = dead(axis(AX_RY, absinfo))
        lt = axis(AX_LT, absinfo)   # 0–1, no deadzone needed for triggers
        rt = axis(AX_RT, absinfo)

        saved = (r, theta, z, pitch, yaw_offset, roll)

        r          += -ly * R_RATE     * DT
        theta      +=  lx * THETA_RATE * DT
        z          += (-btn(BTN_A) + btn(BTN_Y)) * Z_RATE * DT
        pitch      += -ry * PITCH_RATE * DT
        roll       +=  rx * ROLL_RATE  * DT
        yaw_offset += (-btn(BTN_LB) + btn(BTN_RB)) * YAW_RATE * DT

        r          = max(50.0,   min(900.0,  r))
        z          = max(0.0,    min(900.0,  z))
        pitch      = max(-90.0,  min(90.0,   pitch))
        roll       = max(-180.0, min(180.0,  roll))
        yaw_offset = max(-90.0,  min(90.0,   yaw_offset))

        # Pincher — triggers run independently, not reverted on out-of-reach
        pincher_pos += (-lt + rt) * PINCHER_RATE * DT
        pincher_pos  = max(PINCHER_MIN, min(PINCHER_MAX, pincher_pos))
        post_pincher(pincher_pos)

        x   = r * math.cos(theta)
        y   = r * math.sin(theta)
        yaw = math.degrees(theta) + yaw_offset

        result = post_ik(x, y, z, pitch, yaw, roll)
        status = "no response"
        if result:
            status = result.get("status", "?")
            oob = result.get("out_of_range", [])
            if oob:
                status += f" [{', '.join(oob)}]"
            if result.get("status") != "in_reach":
                r, theta, z, pitch, yaw_offset, roll = saved
        else:
            r, theta, z, pitch, yaw_offset, roll = saved

        print(
            f"\r  r={r:5.0f}  θ={math.degrees(theta):6.1f}°  z={z:5.0f}"
            f"  p={pitch:5.1f}°  yaw={yaw:6.1f}°  roll={roll:5.1f}°"
            f"  pincher={pincher_pos:5.0f}"
            f"  | {status:<30}",
            end=""
        )

        elapsed = time.time() - t0
        sleep = DT - elapsed
        if sleep > 0:
            time.sleep(sleep)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
