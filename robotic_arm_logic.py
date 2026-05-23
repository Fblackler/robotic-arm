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
# TODO: Change limits to arm physical limitations.
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



def toPos(SERVOS):
    # To change with conversion angle
    base_angle = math.radians(SERVOS["Base"].position)
    arm_angle = math.radians(SERVOS["Arm"].position)
    elbow_angle = math.radians(SERVOS["Elbow"].position) + arm_angle
    forearm_total_pitch = elbow_angle - math.radians(300) # TODO: To be determined
    forearm_angle = math.radians(SERVOS["Forearm"].position)
    wrist_angle = math.radians(SERVOS["Wrist"].position)
    hand_total_angle = wrist_angle + math.radians(13) #TODO: To be determined
    hand_angle = math.radians(SERVOS["Hand"].position)

    # Lengths in mm
    base_height = 130
    arm_length = 320
    elbow_length = 62
    forearm_length = 325
    wrist_length = 60
    hand_length = 100

    wrist_offset = 10 # upwards

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
    
    # =========================================================
    # Wrist Basis Vectors
    # =========================================================

    # Wrist target position relative to forearm joint
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

    # Forward direction from forearm joint to wrist target
    forearm_dir_x = norm_wrist_pos_x - forearm_pos_x
    forearm_dir_y = norm_wrist_pos_y - forearm_pos_y
    forearm_dir_z = norm_wrist_pos_z - forearm_pos_z

    forearm_dir_length = math.sqrt(
        forearm_dir_x**2 +
        forearm_dir_y**2 +
        forearm_dir_z**2
    )

    forearm_dir_x /= forearm_dir_length
    forearm_dir_y /= forearm_dir_length
    forearm_dir_z /= forearm_dir_length


    # =========================================================
    # Local Up Vector
    # =========================================================

    up_dir_x = math.sin(base_angle) * forearm_dir_z
    up_dir_y = -math.cos(base_angle) * forearm_dir_z

    up_dir_z = (
        math.cos(base_angle) * forearm_dir_y -
        math.sin(base_angle) * forearm_dir_x
    )

    up_dir_length = math.sqrt(
        up_dir_x**2 +
        up_dir_y**2 +
        up_dir_z**2
    )

    up_dir_x /= up_dir_length
    up_dir_y /= up_dir_length
    up_dir_z /= up_dir_length


    # =========================================================
    # Local Side Vector
    # =========================================================

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


    # =========================================================
    # Wrist Roll Offset Direction
    # =========================================================

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


    # =========================================================
    # Wrist Position
    # =========================================================

    wrist_pos_x = (
        norm_wrist_pos_x +
        wrist_offset_vector_x * wrist_offset
    )

    wrist_pos_y = (
        norm_wrist_pos_y +
        wrist_offset_vector_y * wrist_offset
    )

    wrist_pos_z = (
        norm_wrist_pos_z +
        wrist_offset_vector_z * wrist_offset
    )

    wrist_pos = {
        "x": forearm_pos["x"] + wrist_pos_x,
        "y": forearm_pos["y"] + wrist_pos_y,
        "z": forearm_pos["z"] + wrist_pos_z
    }


    # =========================================================
    # Hand Direction
    # =========================================================

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


    # =========================================================
    # Hand Position
    # =========================================================

    hand_pos = {
        "x": wrist_pos["x"] + hand_dir_x * wrist_length,
        "y": wrist_pos["y"] + hand_dir_y * wrist_length,
        "z": wrist_pos["z"] + hand_dir_z * wrist_length
    }

    # =========================================================
    # Pincher Direction
    # =========================================================

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


    # =========================================================
    # Pincher Position
    # =========================================================

    pincher_pos = {
        "x": hand_pos["x"] + pincher_dir_x * hand_length,
        "y": hand_pos["y"] + pincher_dir_y * hand_length,
        "z": hand_pos["z"] + pincher_dir_z * hand_length
    }


