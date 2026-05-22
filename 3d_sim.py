import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D


# =========================================================
# Fake Servo Class
# =========================================================

class FakeServo:
    def __init__(self, limits, default_pos):
        self.limits = limits
        self.position = default_pos


# =========================================================
# Servos
# =========================================================

SERVOS = {
    "Base": FakeServo((0, 270), 145),
    "Arm": FakeServo((50, 170), 135),
    "Elbow": FakeServo((70, 270), 130),
    "Forearm": FakeServo((0, 270), 175),
    "Wrist": FakeServo((30, 270), 80),
    "Hand": FakeServo((0, 270), 140),
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
        elbow_angle - math.radians(65)
    )

    forearm_angle = math.radians(
        SERVOS["Forearm"].position
    )

    wrist_angle = math.radians(
        SERVOS["Wrist"].position
    )

    hand_total_angle = (
        wrist_angle + math.radians(13)
    )

    hand_angle = math.radians(
        SERVOS["Hand"].position
    )


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

    up_dir_x = (
        math.sin(base_angle) *
        forearm_dir_z
    )

    up_dir_y = (
        -math.cos(base_angle) *
        forearm_dir_z
    )

    up_dir_z = (
        math.cos(base_angle) *
        forearm_dir_y -
        math.sin(base_angle) *
        forearm_dir_x
    )

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
        math.cos(wrist_angle) * forearm_dir_x +
        math.sin(wrist_angle) * wrist_offset_vector_x
    )

    hand_dir_y = (
        math.cos(wrist_angle) * forearm_dir_y +
        math.sin(wrist_angle) * wrist_offset_vector_y
    )

    hand_dir_z = (
        math.cos(wrist_angle) * forearm_dir_z +
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


    return [
        base_pos,
        arm_pos,
        elbow_pos,
        forearm_pos,
        wrist_pos,
        hand_pos,
        pincher_pos
    ]


# =========================================================
# Visualization
# =========================================================

fig = plt.figure(figsize=(10, 10))

ax = fig.add_subplot(
    111,
    projection='3d'
)


def update(frame):

    ax.clear()

    joints = toPos(SERVOS)

    xs = [p["x"] for p in joints]
    ys = [p["y"] for p in joints]
    zs = [p["z"] for p in joints]

    # =============================================
    # Draw Robot Arm
    # =============================================

    ax.plot(
        xs,
        ys,
        zs,
        linewidth=3
    )

    ax.scatter(
        xs,
        ys,
        zs,
        s=50
    )

    # =============================================
    # Pincher Point
    # =============================================

    pincher_pos = joints[-1]

    ax.scatter(
        [pincher_pos["x"]],
        [pincher_pos["y"]],
        [pincher_pos["z"]],
        s=100
    )

    # =============================================
    # Ground Plane
    # =============================================

    x = np.array([
        [-900, -900],
        [900, 900]
    ])

    y = np.array([
        [-900, 900],
        [-900, 900]
    ])

    z = np.array([
        [0, 0],
        [0, 0]
    ])

    ax.plot_surface(
        x,
        y,
        z,
        alpha=0.15
    )

    # =============================================
    # Camera Rotation
    # =============================================

    ax.view_init(
        elev=30,
        azim=frame * 0.3
    )

    # =============================================
    # Axes
    # =============================================

    ax.set_xlim(-900, 900)
    ax.set_ylim(-900, 900)
    ax.set_zlim(0, 900)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_title("Robot Arm Default Position")


# =========================================================
# Animation
# =========================================================

ani = FuncAnimation(
    fig,
    update,
    interval=50,
    cache_frame_data=False
)

plt.show()