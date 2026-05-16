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
            default_pos=2042
        ),

    "Base": 
        Servo(
            0,
            "hobby",
            limits=(0, 270),
            default_pos=145
        ),

    "Arm": 
        Servo(
            1,
            "hobby",
            limits=(0, 270),
            default_pos=135
        ),

    "Elbow": 
        Servo(
            2,
            "hobby",
            limits=(0, 270),
            default_pos=130
        ),

    "Forearm": 
        Servo(
            3,
            "hobby",
            limits=(0, 270),
            default_pos=175
        ),

    "Wrist": 
        Servo(
            4,
            "hobby",
            limits=(0, 270),
            default_pos=80
        ),

    "Hand": 
        Servo(
            5,
            "hobby",
            limits=(0, 270),
            default_pos=140
        )
}



def toPos():
    

def main():
    posx, posy, posz = toPos(SERVOS)