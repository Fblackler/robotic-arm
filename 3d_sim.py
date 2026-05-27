import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D


# =========================================================
# Fake Servo Class
# =========================================================

class FakeServo:
    def __init__(self, limits, default_pos, conversion_angle=0):
        self.limits = limits
        self.default_pos = default_pos
        self.conversion_angle = conversion_angle
        self.position = default_pos - conversion_angle

    def write_position(self, position):
        self.position = position - self.conversion_angle


# =========================================================
# Servos
# =========================================================

SERVOS = {
    "Base":    FakeServo((0, 270),   145, conversion_angle=290 - 145),
    "Arm":     FakeServo((50, 170),  100, conversion_angle=127 - 100),
    "Elbow":   FakeServo((70, 270),  140, conversion_angle=247 - 140),
    "Forearm": FakeServo((0, 270),   126, conversion_angle=252 - 126),
    "Wrist":   FakeServo((30, 270),  80,  conversion_angle=225 - 80),
    "Hand":    FakeServo((0, 270),   130, conversion_angle=260 - 130),
}


# =========================================================
# Forward Kinematics (from robotic_arm_logic.py)
# =========================================================

def normalise(vector):
    return vector / np.linalg.norm(vector)


def forward_kinematics(SERVOS):
    base_angle          = math.radians(SERVOS["Base"].position)
    arm_angle           = math.radians(SERVOS["Arm"].position)
    elbow_angle         = math.radians(SERVOS["Elbow"].position) + arm_angle
    forearm_total_pitch = elbow_angle - math.radians(300)
    forearm_angle       = math.radians(SERVOS["Forearm"].position)
    wrist_angle         = math.radians(SERVOS["Wrist"].position)

    base_height    = 130
    arm_length     = 320
    elbow_length   = 62
    forearm_length = 325
    hand_length    = 230

    base_pos = {"x": 0, "y": 0, "z": 0}
    arm_pos  = {"x": 0, "y": 0, "z": base_height}

    elbow_pos = {
        "x": arm_pos["x"] + arm_length * math.cos(arm_angle) * math.cos(base_angle),
        "y": arm_pos["y"] + arm_length * math.cos(arm_angle) * math.sin(base_angle),
        "z": arm_pos["z"] + arm_length * math.sin(arm_angle),
    }

    forearm_pos = {
        "x": elbow_pos["x"] + elbow_length * math.cos(elbow_angle) * math.cos(base_angle),
        "y": elbow_pos["y"] + elbow_length * math.cos(elbow_angle) * math.sin(base_angle),
        "z": elbow_pos["z"] + elbow_length * math.sin(elbow_angle),
    }

    wrist_pos = {
        "x": forearm_pos["x"] + forearm_length * math.cos(forearm_total_pitch) * math.cos(base_angle),
        "y": forearm_pos["y"] + forearm_length * math.cos(forearm_total_pitch) * math.sin(base_angle),
        "z": forearm_pos["z"] + forearm_length * math.sin(forearm_total_pitch),
    }

    forearm_axis = normalise(np.array([
        wrist_pos["x"] - forearm_pos["x"],
        wrist_pos["y"] - forearm_pos["y"],
        wrist_pos["z"] - forearm_pos["z"],
    ]))

    radius = hand_length * math.sin(wrist_angle)
    reach  = hand_length * math.cos(wrist_angle)

    temp = np.array([0, 0, 1])
    if abs(np.dot(temp, forearm_axis)) > 0.99:
        temp = np.array([1, 0, 0])

    right  = normalise(np.cross(temp, forearm_axis))
    up     = np.cross(forearm_axis, right)
    offset = radius * math.cos(forearm_angle) * right + radius * math.sin(forearm_angle) * up
    point  = reach * forearm_axis + offset

    hand_pos = {
        "x": point[0] + wrist_pos["x"],
        "y": point[1] + wrist_pos["y"],
        "z": point[2] + wrist_pos["z"],
    }

    return [base_pos, arm_pos, elbow_pos, forearm_pos, wrist_pos, hand_pos]


# =========================================================
# Visualization
# =========================================================

SERVO_NAMES = ["Base", "Arm", "Elbow", "Forearm", "Wrist", "Hand"]

fig = plt.figure(figsize=(12, 9))
ax  = fig.add_subplot(111, projection='3d')

fig.subplots_adjust(left=0.05, right=0.72)

ax.set_xlim3d(-900, 900)
ax.set_ylim3d(-900, 900)
ax.set_zlim3d(0, 900)


# =========================================================
# Sliders
# =========================================================

sliders = {}

for i, name in enumerate(SERVO_NAMES):
    servo      = SERVOS[name]
    slider_ax  = fig.add_axes([0.76, 0.08 + i * 0.13, 0.20, 0.04])
    slider     = Slider(slider_ax, name, servo.limits[0], servo.limits[1], valinit=servo.default_pos)
    sliders[name] = slider


# =========================================================
# Draw
# =========================================================

def draw(_=None):
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

    x = np.array([[-900, -900], [900, 900]])
    y = np.array([[-900,  900], [-900, 900]])
    z = np.array([[0, 0], [0, 0]])
    ax.plot_surface(x, y, z, alpha=0.15)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Robot Arm")

    labels = [
        f"Base: {SERVOS['Base'].position:.1f}°",
        f"Arm: {SERVOS['Arm'].position:.1f}°",
        f"Elbow: {SERVOS['Elbow'].position:.1f}°",
        f"Forearm: {SERVOS['Forearm'].position:.1f}°",
        f"Wrist: {SERVOS['Wrist'].position:.1f}°",
        f"Hand: {SERVOS['Hand'].position:.1f}°",
    ]

    for joint, label in zip(joints, labels):
        ax.text(joint["x"], joint["y"], joint["z"] + 20, label, fontsize=7, color="black")

    fig.canvas.draw_idle()


UNFLIPPED = {"Base", "Arm", "Forearm"}

def on_slider_change(_):
    for name, slider in sliders.items():
        servo = SERVOS[name]
        if name in UNFLIPPED:
            servo.write_position(slider.val)
        else:
            servo.write_position(2 * servo.default_pos - slider.val)
    draw()


for slider in sliders.values():
    slider.on_changed(on_slider_change)


draw()
plt.show()
