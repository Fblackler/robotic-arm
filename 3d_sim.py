import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

from robotic_arm_logic import forward_kinematics, inverse_kinematics, SERVOS as _real_servos


# =========================================================
# Fake Servo
# =========================================================

class FakeServo:
    def __init__(self, limits, default_pos, conversion_angle=0):
        self.limits        = limits
        self.default_pos   = default_pos
        self.conversion_angle = conversion_angle
        self.position      = conversion_angle

    def write_position(self, cmd):
        self.position = self.conversion_angle + (cmd - self.default_pos)


# =========================================================
# Servos — mirrored from robotic_arm_logic
# =========================================================

SERVO_NAMES = ["Base", "Arm", "Elbow", "Forearm", "Wrist", "Hand"]

SERVOS = {
    name: FakeServo(
        _real_servos[name].limits,
        _real_servos[name].default_pos,
        conversion_angle=_real_servos[name].conversion_angle,
    )
    for name in SERVO_NAMES
}


# =========================================================
# IK helpers
# =========================================================

def kin_to_cmd(servo, kin_deg):
    return kin_deg - servo.conversion_angle + servo.default_pos

def in_limits(servo, cmd):
    return servo.limits[0] <= cmd <= servo.limits[1]


# =========================================================
# Figure
# =========================================================

fig = plt.figure(figsize=(14, 9))
ax  = fig.add_subplot(111, projection='3d')
fig.subplots_adjust(left=0.05, right=0.68)

ax.set_xlim3d(-900, 900)
ax.set_ylim3d(-900, 900)
ax.set_zlim3d(0, 900)
ax.set_box_aspect([2, 2, 1])


# =========================================================
# IK Target Sliders
# =========================================================

TARGET_PARAMS = [
    ("X",     -700,  700,  300),
    ("Y",     -700,  700,    0),
    ("Z",        0,  700,  300),
    ("Pitch",  -90,   90,  -20),
    ("Yaw",   -180,  180,    0),
]

sliders = {}
for i, (name, lo, hi, init) in enumerate(TARGET_PARAMS):
    sax = fig.add_axes([0.72, 0.08 + i * 0.13, 0.24, 0.04])
    sliders[name] = Slider(sax, name, lo, hi, valinit=init)


# =========================================================
# Info text panel
# =========================================================

info_ax = fig.add_axes([0.70, 0.72, 0.28, 0.24])
info_ax.axis('off')
info_text = info_ax.text(
    0, 1, "", transform=info_ax.transAxes,
    fontsize=7.5, verticalalignment='top', family='monospace'
)


# =========================================================
# Draw
# =========================================================

def draw(status=""):
    elev = ax.elev
    azim = ax.azim
    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()
    ax.clear()
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlim3d(xlim)
    ax.set_ylim3d(ylim)
    ax.set_zlim3d(zlim)

    joints = forward_kinematics(SERVOS)
    xs = [p["x"] for p in joints]
    ys = [p["y"] for p in joints]
    zs = [p["z"] for p in joints]

    ax.plot(xs, ys, zs, linewidth=3)
    ax.scatter(xs, ys, zs, s=50)
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], s=100)

    # Target marker
    tx, ty, tz = sliders["X"].val, sliders["Y"].val, sliders["Z"].val
    color = "red" if "OUT" in status else "green"
    ax.scatter([tx], [ty], [tz], s=150, c=color, marker="x", zorder=5, linewidths=2)

    x_grid = np.array([[-900, -900], [900,  900]])
    y_grid = np.array([[-900,  900], [-900, 900]])
    z_grid = np.zeros((2, 2))
    ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.15)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Robot Arm  —  {status}" if status else "Robot Arm")

    labels = [f"{n}: {SERVOS[n].position:.1f}°" for n in SERVO_NAMES]
    for joint, label in zip(joints, labels):
        ax.text(joint["x"], joint["y"], joint["z"] + 20, label, fontsize=7, color="black")

    fig.canvas.draw_idle()


# =========================================================
# IK Callback
# =========================================================

def on_change(_):
    target_pos   = {"x": sliders["X"].val, "y": sliders["Y"].val, "z": sliders["Z"].val}
    target_pitch = math.radians(sliders["Pitch"].val)
    target_yaw   = math.radians(sliders["Yaw"].val)

    try:
        ik = inverse_kinematics(target_pos, target_pitch, target_yaw)
    except Exception as e:
        info_text.set_text(f"IK error:\n{e}")
        draw("ERROR")
        return

    # Convert kinematic → servo cmd, check limits
    out_of_range = []
    cmds = {}
    for name in SERVO_NAMES:
        if name not in ik:
            continue
        servo = SERVOS[name]
        kin   = ik[name]
        cmd   = kin_to_cmd(servo, kin)
        cmds[name] = (kin, cmd)
        if not in_limits(servo, cmd):
            out_of_range.append(name)

    # Info panel lines
    lines = []
    for name in SERVO_NAMES:
        if name not in cmds:
            continue
        kin, cmd = cmds[name]
        servo = SERVOS[name]
        ok    = in_limits(servo, cmd)
        mark  = " " if ok else "!"
        lines.append(
            f"[{mark}] {name:<8} {kin:+7.1f}°  cmd:{cmd:5.0f}°"
            f"  [{servo.limits[0]}-{servo.limits[1]}]"
        )

    if out_of_range:
        status = f"OUT OF REACH: {', '.join(out_of_range)}"
        info_text.set_text(status + "\n\n" + "\n".join(lines))
        draw("OUT OF REACH")
    else:
        for name in SERVO_NAMES:
            if name in cmds:
                SERVOS[name].write_position(cmds[name][1])
        info_text.set_text("IN REACH\n\n" + "\n".join(lines))
        draw("IN REACH")


for slider in sliders.values():
    slider.on_changed(on_change)

draw()
on_change(None)
plt.show()
