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
            default_pos=138,
            conversion_angle=0
        ),

    "Arm": 
        Servo(
            1,
            "hobby",
            limits=(50, 170),
            default_pos=100,
            conversion_angle=112
        ),

    "Elbow": 
        Servo(
            2,
            "hobby",
            limits=(70, 270),
            default_pos=140,
            conversion_angle=-44
        ),

    "Forearm":
        Servo(
            3,
            "hobby",
            limits=(0, 270),
            default_pos=126,
            conversion_angle=90
        ),

    "Wrist": 
        Servo(
            4,
            "hobby",
            limits=(30, 235),
            default_pos=80,
            conversion_angle=-40
        ),

    "Hand": 
        Servo(
            5,
            "hobby",
            limits=(0, 270),
            default_pos=130,
            conversion_angle=0
        )
}

# Lengths in mm
base_height = 130
arm_length = 320
elbow_length = 62
forearm_length = 325
hand_length = 220

def normalise(vector):
        return vector / np.linalg.norm(vector)

def forward_kinematics(SERVOS):
    # To change with conversion angle
    base_angle = math.radians(SERVOS["Base"].position)
    arm_angle = math.radians(SERVOS["Arm"].position)
    elbow_angle = math.radians(SERVOS["Elbow"].position) + arm_angle
    forearm_total_pitch = elbow_angle - math.radians(70)
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

    right = np.array([-math.sin(base_angle), math.cos(base_angle), 0.0])
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

    return [base_pos, arm_pos, elbow_pos, forearm_pos, wrist_pos, hand_pos]


def inverse_kinematics(target_pos, target_pitch, target_yaw, target_roll=SERVOS["Hand"].position):

    target_roll = math.radians(target_roll)

    wrist_pos = {
        "x": target_pos["x"] - hand_length * math.cos(target_pitch) * math.cos(target_yaw),
        "y": target_pos["y"] - hand_length * math.cos(target_pitch) * math.sin(target_yaw),
        "z": target_pos["z"] - hand_length * math.sin(target_pitch),
    }

    base_angle = math.atan2(wrist_pos["y"], wrist_pos["x"])
    base_pos = {"x": 0, "y": 0, "z": 0}
    arm_pos = {"x": 0, "y": 0, "z": base_height}

    (score_a, sol_a), (score_b, sol_b) = optimal_arm_to_forearm_angle(wrist_pos, base_angle, arm_pos)

    GOOD_ENOUGH = 20  # mm — maximum acceptable wrist position error

    def forearm_angle_for(arm_ang, elbow_ang):
        ar = math.radians(arm_ang)
        er = math.radians(elbow_ang) + ar
        ep = {
            "x": arm_pos["x"] + arm_length * math.cos(ar) * math.cos(base_angle),
            "y": arm_pos["y"] + arm_length * math.cos(ar) * math.sin(base_angle),
            "z": arm_pos["z"] + arm_length * math.sin(ar),
        }
        fp = {
            "x": ep["x"] + elbow_length * math.cos(er) * math.cos(base_angle),
            "y": ep["y"] + elbow_length * math.cos(er) * math.sin(base_angle),
            "z": ep["z"] + elbow_length * math.sin(er),
        }
        fax = normalise(np.array([
            wrist_pos["x"] - fp["x"],
            wrist_pos["y"] - fp["y"],
            wrist_pos["z"] - fp["z"],
        ]))
        r = np.array([-math.sin(base_angle), math.cos(base_angle), 0.0])
        u = np.cross(fax, r)
        td = np.array([
            math.cos(target_pitch) * math.cos(target_yaw),
            math.cos(target_pitch) * math.sin(target_yaw),
            math.sin(target_pitch),
        ])
        return math.atan2(np.dot(td, u), np.dot(td, r))

    fa_a = forearm_angle_for(*sol_a)
    fa_b = forearm_angle_for(*sol_b)

    both_valid = score_a < GOOD_ENOUGH and score_b < GOOD_ENOUGH
    if both_valid:
        arm_angle, elbow_angle = sol_a if abs(fa_a) <= abs(fa_b) else sol_b
    elif score_a <= score_b:
        arm_angle, elbow_angle = sol_a
    else:
        arm_angle, elbow_angle = sol_b

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

    right = np.array([-math.sin(base_angle), math.cos(base_angle), 0.0])
    up    = np.cross(forearm_axis, right)

    target_dir = np.array([
        math.cos(target_pitch) * math.cos(target_yaw),
        math.cos(target_pitch) * math.sin(target_yaw),
        math.sin(target_pitch)
    ])

    fa        = math.atan2(np.dot(target_dir, up), np.dot(target_dir, right))
    wrist_angle = math.acos(np.clip(np.dot(target_dir, forearm_axis), -1.0, 1.0))

    # Two equivalent options: (fa, wa) and (fa+180°, -wa).
    # Both achieve the same hand direction — pick whichever servo commands are in range,
    # preferring the one closest to the forearm's default position.
    fa_b = fa + math.pi
    if fa_b > math.pi:
        fa_b -= 2 * math.pi

    _fs = SERVOS["Forearm"]
    _ws = SERVOS["Wrist"]

    def _cmd(sdef, kin_deg):
        return kin_deg - sdef.conversion_angle + sdef.default_pos

    def _ok(sdef, kin_deg):
        c = _cmd(sdef, kin_deg)
        return sdef.limits[0] <= c <= sdef.limits[1]

    ok_a = _ok(_fs, math.degrees(fa))   and _ok(_ws,  math.degrees(wrist_angle))
    ok_b = _ok(_fs, math.degrees(fa_b)) and _ok(_ws, -math.degrees(wrist_angle))

    if ok_a and ok_b:
        effort_a = abs(_cmd(_fs, math.degrees(fa))   - _fs.default_pos)
        effort_b = abs(_cmd(_fs, math.degrees(fa_b)) - _fs.default_pos)
        forearm_angle = fa   if effort_a <= effort_b else fa_b
        wrist_angle   = wrist_angle if effort_a <= effort_b else -wrist_angle
    elif ok_a:
        forearm_angle = fa
    elif ok_b:
        forearm_angle = fa_b
        wrist_angle   = -wrist_angle
    else:
        forearm_angle = fa  # both out of range — let limit check report it

    tilt_dir  =  math.cos(forearm_angle) * right + math.sin(forearm_angle) * up
    target_up = -math.sin(target_roll)   * right + math.cos(target_roll)   * up
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
    arm_s   = SERVOS["Arm"]
    elbow_s = SERVOS["Elbow"]

    arm_kin_min   = round(arm_s.conversion_angle   + (arm_s.limits[0]   - arm_s.default_pos))
    arm_kin_max   = round(arm_s.conversion_angle   + (arm_s.limits[1]   - arm_s.default_pos))
    elbow_kin_min = round(elbow_s.conversion_angle + (elbow_s.limits[0] - elbow_s.default_pos))
    elbow_kin_max = round(elbow_s.conversion_angle + (elbow_s.limits[1] - elbow_s.default_pos))

    # Track best solution per elbow branch — store (score, angles)
    best_neg = [float("inf"), None]  # elbow_angle < 0
    best_pos = [float("inf"), None]  # elbow_angle >= 0

    for arm_angle in range(arm_kin_min, arm_kin_max + 1, 2):
        arm_angle_r = math.radians(arm_angle)
        elbow_pos = {
            "x": arm_pos["x"] + arm_length * math.cos(arm_angle_r) * math.cos(base_angle),
            "y": arm_pos["y"] + arm_length * math.cos(arm_angle_r) * math.sin(base_angle),
            "z": arm_pos["z"] + arm_length * math.sin(arm_angle_r),
        }
        for elbow_angle in range(elbow_kin_min, elbow_kin_max + 1, 2):
            elbow_angle_r = math.radians(elbow_angle) + arm_angle_r
            forearm_pos = {
                "x": elbow_pos["x"] + elbow_length * math.cos(elbow_angle_r) * math.cos(base_angle),
                "y": elbow_pos["y"] + elbow_length * math.cos(elbow_angle_r) * math.sin(base_angle),
                "z": elbow_pos["z"] + elbow_length * math.sin(elbow_angle_r),
            }
            forearm_total_pitch = elbow_angle_r - math.radians(70)
            sup_wrist = {
                "x": forearm_pos["x"] + forearm_length * math.cos(forearm_total_pitch) * math.cos(base_angle),
                "y": forearm_pos["y"] + forearm_length * math.cos(forearm_total_pitch) * math.sin(base_angle),
                "z": forearm_pos["z"] + forearm_length * math.sin(forearm_total_pitch),
            }
            score = (
                abs(sup_wrist["x"] - wrist_pos["x"]) +
                abs(sup_wrist["y"] - wrist_pos["y"]) +
                abs(sup_wrist["z"] - wrist_pos["z"])
            )
            bucket = best_neg if elbow_angle < 0 else best_pos
            if score < bucket[0]:
                bucket[0] = score
                bucket[1] = (arm_angle, elbow_angle)

    # Fall back to the other branch if one found nothing
    if best_neg[1] is None:
        best_neg[1] = best_pos[1]
    if best_pos[1] is None:
        best_pos[1] = best_neg[1]

    return (best_neg[0], best_neg[1]), (best_pos[0], best_pos[1])