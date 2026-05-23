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
    "Wrist":   FakeServo((30, 270),  80,  conversion_angle=123 - 80),
    "Hand":    FakeServo((0, 270),   130, conversion_angle=260 - 130),
}


# =========================================================
# Forward Kinematics
# =========================================================

def toPos(SERVOS):

    # =====================================================
    # Angles
    # =====================================================

    base_angle = math.radians(
        SERVOS["Base"].position
    )

    arm_angle = math.radians(
        SERVOS["Arm"].position
    )

    elbow_angle = (
        math.radians(SERVOS["Elbow"].position) +
        arm_angle
    )

    forearm_total_pitch = (
        elbow_angle - math.radians(300)
    )

    forearm_angle = math.radians(
        SERVOS["Forearm"].position
    )

    wrist_angle = math.radians(
        SERVOS["Wrist"].position
    )

    hand_angle = math.radians(
        SERVOS["Hand"].position
    )

    hand_total_angle = math.radians(13)


    # =====================================================
    # Lengths
    # =====================================================

    base_height = 130
    arm_length = 320
    elbow_length = 62
    forearm_length = 325
    wrist_length = 60
    hand_length = 100

    wrist_offset = 10


    # =====================================================
    # Base Position
    # =====================================================

    base_pos = {
        "x": 0,
        "y": 0,
        "z": 0
    }


    # =====================================================
    # Arm Position
    # =====================================================

    arm_pos = {
        "x": 0,
        "y": 0,
        "z": base_height
    }


    # =====================================================
    # Elbow Position
    # =====================================================

    elbow_pos_x = (
        arm_length *
        math.cos(arm_angle) *
        math.cos(base_angle)
    )

    elbow_pos_y = (
        arm_length *
        math.cos(arm_angle) *
        math.sin(base_angle)
    )

    elbow_pos_z = (
        arm_length *
        math.sin(arm_angle)
    )

    elbow_pos = {
        "x": arm_pos["x"] + elbow_pos_x,
        "y": arm_pos["y"] + elbow_pos_y,
        "z": arm_pos["z"] + elbow_pos_z
    }


    # =====================================================
    # Forearm Joint Position
    # =====================================================

    forearm_pos_x = (
        elbow_length *
        math.cos(elbow_angle) *
        math.cos(base_angle)
    )

    forearm_pos_y = (
        elbow_length *
        math.cos(elbow_angle) *
        math.sin(base_angle)
    )

    forearm_pos_z = (
        elbow_length *
        math.sin(elbow_angle)
    )

    forearm_pos = {
        "x": elbow_pos["x"] + forearm_pos_x,
        "y": elbow_pos["y"] + forearm_pos_y,
        "z": elbow_pos["z"] + forearm_pos_z
    }


    # =====================================================
    # Wrist Neutral Position
    # =====================================================

    norm_wrist_pos_x = (
        forearm_length *
        math.cos(forearm_total_pitch) *
        math.cos(base_angle)
    )

    norm_wrist_pos_y = (
        forearm_length *
        math.cos(forearm_total_pitch) *
        math.sin(base_angle)
    )

    norm_wrist_pos_z = (
        forearm_length *
        math.sin(forearm_total_pitch)
    )


    # =====================================================
    # Forearm Direction
    # =====================================================

    forearm_dir_x = (
        norm_wrist_pos_x - forearm_pos_x
    )

    forearm_dir_y = (
        norm_wrist_pos_y - forearm_pos_y
    )

    forearm_dir_z = (
        norm_wrist_pos_z - forearm_pos_z
    )

    forearm_dir_length = math.sqrt(
        forearm_dir_x**2 +
        forearm_dir_y**2 +
        forearm_dir_z**2
    )

    forearm_dir_x /= forearm_dir_length
    forearm_dir_y /= forearm_dir_length
    forearm_dir_z /= forearm_dir_length


    # =====================================================
    # Up Direction
    # =====================================================

    dot = forearm_dir_z

    up_dir_x = -forearm_dir_x * dot
    up_dir_y = -forearm_dir_y * dot
    up_dir_z = 1.0 - forearm_dir_z * dot

    up_dir_length = math.sqrt(
        up_dir_x**2 +
        up_dir_y**2 +
        up_dir_z**2
    )

    up_dir_x /= up_dir_length
    up_dir_y /= up_dir_length
    up_dir_z /= up_dir_length


    # =====================================================
    # Side Direction
    # =====================================================

    side_dir_x = (
        forearm_dir_y * up_dir_z -
        forearm_dir_z * up_dir_y
    )

    side_dir_y = (
        forearm_dir_z * up_dir_x -
        forearm_dir_x * up_dir_z
    )

    side_dir_z = (
        forearm_dir_x * up_dir_y -
        forearm_dir_y * up_dir_x
    )


    # =====================================================
    # Wrist Offset Vector
    # =====================================================

    wrist_offset_vector_x = (
        math.cos(forearm_angle) * up_dir_x +
        math.sin(forearm_angle) * side_dir_x
    )

    wrist_offset_vector_y = (
        math.cos(forearm_angle) * up_dir_y +
        math.sin(forearm_angle) * side_dir_y
    )

    wrist_offset_vector_z = (
        math.cos(forearm_angle) * up_dir_z +
        math.sin(forearm_angle) * side_dir_z
    )


    # =====================================================
    # Wrist Position
    # =====================================================

    wrist_pos = {
        "x":
            forearm_pos["x"] +
            norm_wrist_pos_x +
            wrist_offset_vector_x * wrist_offset,

        "y":
            forearm_pos["y"] +
            norm_wrist_pos_y +
            wrist_offset_vector_y * wrist_offset,

        "z":
            forearm_pos["z"] +
            norm_wrist_pos_z +
            wrist_offset_vector_z * wrist_offset
    }


    # =====================================================
    # Hand Direction
    # =====================================================

    hand_dir_x = (
        math.cos(wrist_angle) * forearm_dir_x -
        math.sin(wrist_angle) * wrist_offset_vector_x
    )

    hand_dir_y = (
        math.cos(wrist_angle) * forearm_dir_y -
        math.sin(wrist_angle) * wrist_offset_vector_y
    )

    hand_dir_z = (
        math.cos(wrist_angle) * forearm_dir_z -
        math.sin(wrist_angle) * wrist_offset_vector_z
    )


    # =====================================================
    # Hand Position
    # =====================================================

    hand_pos = {
        "x": wrist_pos["x"] + hand_dir_x * wrist_length,
        "y": wrist_pos["y"] + hand_dir_y * wrist_length,
        "z": wrist_pos["z"] + hand_dir_z * wrist_length
    }


    # =====================================================
    # Pincher Direction
    # =====================================================

    pincher_dir_x = (
        math.cos(hand_total_angle) * hand_dir_x +
        math.sin(hand_total_angle) * wrist_offset_vector_x
    )

    pincher_dir_y = (
        math.cos(hand_total_angle) * hand_dir_y +
        math.sin(hand_total_angle) * wrist_offset_vector_y
    )

    pincher_dir_z = (
        math.cos(hand_total_angle) * hand_dir_z +
        math.sin(hand_total_angle) * wrist_offset_vector_z
    )


    # =====================================================
    # Pincher Position
    # =====================================================

    pincher_pos = {
        "x": hand_pos["x"] + pincher_dir_x * hand_length,
        "y": hand_pos["y"] + pincher_dir_y * hand_length,
        "z": hand_pos["z"] + pincher_dir_z * hand_length
    }


    norm_wrist_axis_pos = {
        "x": forearm_pos["x"] + norm_wrist_pos_x,
        "y": forearm_pos["y"] + norm_wrist_pos_y,
        "z": forearm_pos["z"] + norm_wrist_pos_z
    }

    return [
        base_pos,
        arm_pos,
        elbow_pos,
        forearm_pos,
        norm_wrist_axis_pos,
        wrist_pos,
        hand_pos,
        pincher_pos
    ]


# =========================================================
# Visualization
# =========================================================

SERVO_NAMES = ["Base", "Arm", "Elbow", "Forearm", "Wrist", "Hand"]

fig = plt.figure(figsize=(12, 9))

ax = fig.add_subplot(
    111,
    projection='3d'
)

fig.subplots_adjust(left=0.05, right=0.65)

ax.set_xlim3d(-900, 900)
ax.set_ylim3d(-900, 900)
ax.set_zlim3d(0, 900)


# =========================================================
# Sliders
# =========================================================

sliders = {}

for i, name in enumerate(SERVO_NAMES):
    servo = SERVOS[name]
    slider_ax = fig.add_axes([0.70, 0.08 + i * 0.13, 0.25, 0.04])
    slider = Slider(
        slider_ax,
        name,
        servo.limits[0],
        servo.limits[1],
        valinit=servo.default_pos
    )
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

    joints = toPos(SERVOS)

    xs = [p["x"] for p in joints]
    ys = [p["y"] for p in joints]
    zs = [p["z"] for p in joints]

    ax.plot(xs, ys, zs, linewidth=3)
    ax.scatter(xs, ys, zs, s=50)

    pincher_pos = joints[-1]
    ax.scatter(
        [pincher_pos["x"]],
        [pincher_pos["y"]],
        [pincher_pos["z"]],
        s=100
    )

    x = np.array([[-900, -900], [900, 900]])
    y = np.array([[-900,  900], [-900, 900]])
    z = np.array([[0, 0], [0, 0]])
    ax.plot_surface(x, y, z, alpha=0.15)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Robot Arm")

    fig.canvas.draw_idle()


def on_slider_change(_):
    for name, slider in sliders.items():
        servo = SERVOS[name]
        flipped = 2 * servo.default_pos - slider.val
        servo.write_position(flipped)
    draw()


for slider in sliders.values():
    slider.on_changed(on_slider_change)


draw()
plt.show()