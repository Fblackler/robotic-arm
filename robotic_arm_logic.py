import numpy as np

"""
SERVO CONTROL SYSTEM

Smart servos:
    servo.write_position(2048)

Position range:
    0-4095

Read smart servo data:
    servo.read_data(REGISTERS["current_position"])
    servo.read_data(REGISTERS["temperature"])
    servo.read_data(REGISTERS["voltage"])

Hobby servos:
    servo.write_position(135)

Range:
    0-270 degrees

Hobby servos return the last commanded position only:
    servo.read_data()
"""


from servo import Servo, REGISTERS
import math


# ====================================================================
# Joints
# ====================================================================
SERVOS = {
    "Pincher":
        Servo(
            1,
            "smart",
            limits=(0, 4095),
            default_pos=2042,
            conversion_angle=0
        ),

    "Base": 
        Servo(
            0,
            "hobby",
            limits=(0, 270),
            default_pos=145,
            conversion_angle=290 - 145
        ),

    "Arm": 
        Servo(
            1,
            "hobby",
            limits=(50, 170),
            default_pos=100,
            conversion_angle=127 - 100
        ),

    "Elbow": 
        Servo(
            2,
            "hobby",
            limits=(70, 270),
            default_pos=140,
            conversion_angle=247 - 140
        ),

    "Forearm": 
        Servo(
            3,
            "hobby",
            limits=(0, 270),
            default_pos=126,
            conversion_angle=252 - 126
        ),

    "Wrist": 
        Servo(
            4,
            "hobby",
            limits=(30, 270),
            default_pos=80,
            conversion_angle=225 - 80
        ),

    "Hand": 
        Servo(
            5,
            "hobby",
            limits=(0, 270),
            default_pos=130,
            conversion_angle=260 - 130
        )
}

# Lengths in mm
base_height = 130
arm_length = 320
elbow_length = 62
forearm_length = 325
hand_length = 230

def normalise(vector):
        return vector / np.linalg.norm(vector)

def forward_kinematics(SERVOS):
    # To change with conversion angle
    base_angle = math.radians(SERVOS["Base"].position)
    arm_angle = math.radians(SERVOS["Arm"].position)
    elbow_angle = math.radians(SERVOS["Elbow"].position) + arm_angle
    forearm_total_pitch = elbow_angle - math.radians(300)
    forearm_angle = math.radians(SERVOS["Forearm"].position)
    wrist_angle = math.radians(SERVOS["Wrist"].position)
    hand_angle = math.radians(SERVOS["Hand"].position)

    base_pos = {
        "x": 0,
        "y":0, 
        "z":0
    }

    arm_pos = {
        "x": base_pos["x"],
        "y": base_pos["y"],
        "z": base_pos["z"] + base_height
    }

    elbow_pos_x = arm_length * math.cos(arm_angle) * math.cos(base_angle)
    elbow_pos_y = arm_length * math.cos(arm_angle) * math.sin(base_angle)
    elbow_pos_z = arm_length * math.sin(arm_angle)
    elbow_pos = {
        "x": arm_pos["x"] + elbow_pos_x, 
        "y": arm_pos["y"] + elbow_pos_y, 
        "z": arm_pos["z"] + elbow_pos_z
    }

    forearm_pos_x = elbow_length * math.cos(elbow_angle) * math.cos(base_angle)
    forearm_pos_y = elbow_length * math.cos(elbow_angle) * math.sin(base_angle)
    forearm_pos_z = elbow_length * math.sin(elbow_angle)
    forearm_pos = {
        "x": elbow_pos["x"] + forearm_pos_x,
        "y": elbow_pos["y"] + forearm_pos_y,
        "z": elbow_pos["z"] + forearm_pos_z
    }

    wrist_pos_x = forearm_length * math.cos(forearm_total_pitch) * math.cos(base_angle)
    wrist_pos_y = forearm_length * math.cos(forearm_total_pitch) * math.sin(base_angle)
    wrist_pos_z = forearm_length * math.sin(forearm_total_pitch)
    wrist_pos = {
        "x": forearm_pos["x"] + wrist_pos_x,
        "y": forearm_pos["y"] + wrist_pos_y,
        "z": forearm_pos["z"] + wrist_pos_z
    }
    
    # Wrist to hand calculation
    forearm_axis = normalise(np.array([
        wrist_pos["x"] - forearm_pos["x"],
        wrist_pos["y"] - forearm_pos["y"],
        wrist_pos["z"] - forearm_pos["z"]
    ]))

    radius = hand_length * math.sin(wrist_angle)
    reach = hand_length * math.cos(wrist_angle)

    temp = np.array([0, 0, 1])

    if abs(np.dot(temp, forearm_axis)) > 0.99:
        temp = np.array([1, 0, 0])

    right = normalise(np.cross(temp, forearm_axis))
    up = np.cross(forearm_axis, right)

    offset = (
          radius * math.cos(forearm_angle) * right +
          radius * math.sin(forearm_angle) * up
    )

    forward = reach * forearm_axis
    point = forward + offset
    hand_pos = {
        "x": point[0] + wrist_pos["x"],
        "y": point[1] + wrist_pos["y"],
        "z": point[2] + wrist_pos["z"]
    }


def inverse_kinematics(target_pos, target_pitch, target_yaw, target_roll=SERVOS["Hand"].position):

    wrist_pos_x = hand_length * math.cos(target_pitch - math.radians(180)) * math.cos(target_yaw - math.radians(180))
    wrist_pos_y = hand_length * math.cos(target_pitch - math.radians(180)) * math.sin(target_yaw - math.radians(180))
    wrist_pos_z = hand_length * math.sin(target_pitch - math.radians(180))
    wrist_pos = {
        "x": target_pos["x"] + wrist_pos_x,
        "y": target_pos["y"] + wrist_pos_y,
        "z": target_pos["z"] + wrist_pos_z
    }

    base_angle = math.atan2(wrist_pos["y"], wrist_pos["x"])
    base_pos = {"x": 0, "y": 0, "z": 0}
    arm_pos = {"x": 0, "y": 0, "z": base_height}
    arm_angle, elbow_angle = optimal_arm_to_forearm_angle(wrist_pos, base_angle, arm_pos)

    arm_angle_r   = math.radians(arm_angle)
    elbow_angle_r = math.radians(elbow_angle) + arm_angle_r

    elbow_pos = {
        "x": arm_pos["x"] + arm_length * math.cos(arm_angle_r) * math.cos(base_angle),
        "y": arm_pos["y"] + arm_length * math.cos(arm_angle_r) * math.sin(base_angle),
        "z": arm_pos["z"] + arm_length * math.sin(arm_angle_r)
    }

    forearm_pos = {
        "x": elbow_pos["x"] + elbow_length * math.cos(elbow_angle_r) * math.cos(base_angle),
        "y": elbow_pos["y"] + elbow_length * math.cos(elbow_angle_r) * math.sin(base_angle),
        "z": elbow_pos["z"] + elbow_length * math.sin(elbow_angle_r)
    }

    forearm_axis = normalise(np.array([
        wrist_pos["x"] - forearm_pos["x"],
        wrist_pos["y"] - forearm_pos["y"],
        wrist_pos["z"] - forearm_pos["z"]
    ]))

    temp = np.array([0, 0, 1])
    if abs(np.dot(temp, forearm_axis)) > 0.99:
        temp = np.array([1, 0, 0])

    right = normalise(np.cross(temp, forearm_axis))
    up    = np.cross(forearm_axis, right)

    target_dir = np.array([
        math.cos(target_pitch) * math.cos(target_yaw),
        math.cos(target_pitch) * math.sin(target_yaw),
        math.sin(target_pitch)
    ])

    forearm_angle = math.atan2(np.dot(target_dir, up), np.dot(target_dir, right))

    wrist_angle = math.acos(np.clip(np.dot(target_dir, forearm_axis), -1.0, 1.0))

    tilt_dir  = math.cos(forearm_angle) * right + math.sin(forearm_angle) * up
    target_up = math.cos(target_roll)   * right + math.sin(target_roll)   * up
    hand_angle = math.atan2(
        np.dot(np.cross(tilt_dir, target_up), target_dir),
        np.dot(tilt_dir, target_up)
    )

    return {
        "Base":    math.degrees(base_angle),
        "Arm":     arm_angle,
        "Elbow":   elbow_angle,
        "Forearm": math.degrees(forearm_angle),
        "Wrist":   math.degrees(wrist_angle),
        "Hand":    math.degrees(hand_angle),
    }


def optimal_arm_to_forearm_angle(wrist_pos, base_angle, arm_pos):
    arm_angles  = range(50, 171)
    elbow_angles = range(70, 271)
    angles = {}

    for i, arm_angle in enumerate(arm_angles):
        arm_angle_r = math.radians(arm_angle)
        elbow_pos_x = arm_length * math.cos(arm_angle_r) * math.cos(base_angle)
        elbow_pos_y = arm_length * math.cos(arm_angle_r) * math.sin(base_angle)
        elbow_pos_z = arm_length * math.sin(arm_angle_r)
        elbow_pos = {
            "x": arm_pos["x"] + elbow_pos_x,
            "y": arm_pos["y"] + elbow_pos_y,
            "z": arm_pos["z"] + elbow_pos_z
        }

        for j, elbow_angle in enumerate(elbow_angles):
            elbow_angle_r = math.radians(elbow_angle) + arm_angle_r
            forearm_pos_x = elbow_length * math.cos(elbow_angle_r) * math.cos(base_angle)
            forearm_pos_y = elbow_length * math.cos(elbow_angle_r) * math.sin(base_angle)
            forearm_pos_z = elbow_length * math.sin(elbow_angle_r)
            forearm_pos = {
                "x": elbow_pos["x"] + forearm_pos_x,
                "y": elbow_pos["y"] + forearm_pos_y,
                "z": elbow_pos["z"] + forearm_pos_z
            }

            forearm_total_pitch = elbow_angle_r - math.radians(300)
            sup_wrist_pos_x = forearm_length * math.cos(forearm_total_pitch) * math.cos(base_angle)
            sup_wrist_pos_y = forearm_length * math.cos(forearm_total_pitch) * math.sin(base_angle)
            sup_wrist_pos_z = forearm_length * math.sin(forearm_total_pitch)
            sup_wrist_pos = {
                "x": forearm_pos["x"] + sup_wrist_pos_x,
                "y": forearm_pos["y"] + sup_wrist_pos_y,
                "z": forearm_pos["z"] + sup_wrist_pos_z
            }
            total_sup_position = (
                  abs(sup_wrist_pos["x"])
                + abs(sup_wrist_pos["y"])
                + abs(sup_wrist_pos["z"])
            )
            total_wrist_position = (
                  abs(wrist_pos["x"])
                + abs(wrist_pos["y"])
                + abs(wrist_pos["z"])
            )
            score = total_sup_position - total_wrist_position
            if score < 5.0:
                continue
            else:
                angles[f"{score}"] = (arm_angle, elbow_angle)

    best_score = min(angles.keys())
    best_arm_angle, best_elbow_angle = angles[best_score]
    return best_arm_angle, best_elbow_angle