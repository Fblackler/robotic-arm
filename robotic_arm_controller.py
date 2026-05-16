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

import tkinter as tk
import serial

# ==========================================================
# Settings
# ==========================================================
# Ports
SMART_PORT = "/dev/cu.usbmodem5B141156511"
HOBBY_PORT = "/dev/tty.usbserial-10"

# Smart Servo settings
SMART_BAUD = 1000000
SMART_SERVO_ID = 1
# Hobby Servo settings
HOBBY_BAUD = 115200

# Open serial connection
smart_ser = serial.Serial(SMART_PORT, SMART_BAUD, timeout=1)
ser = serial.Serial(HOBBY_PORT, HOBBY_BAUD, timeout=1)

# Smart servo range
SMART_MIN_POS = 0
SMART_MAX_POS = 4095

# Smart servo registers:
REGISTERS = {
    "goal_position": (0x2A, 2),
    "current_position": (0x38, 2),
    "speed": (0x3A, 2),
    "load": (0x3C, 2),
    "voltage": (0x3E, 1),
    "temperature": (0x3F, 1),
}

# ==========================================================
# Servo Class
# ==========================================================
class Servo:
    def __init__(self, servo_id, servo_type):
        self.id = servo_id
        self.type = servo_type
        self.position = None

        if self.type == "Smart":
            self.ser = smart_ser
        else:
            self.ser = ser

    def write_position(self, position):

        if self.type == "Smart":
            return smart_write_position(self, position)
        else:
            self.position = position
            return hobby_write_position(self, position)
            

    def read_data(self, register_address=None):

        register, size = register_address

        if self.type == "Smart":
            return smart_read(self, register, size)
        else:
            return self.position
        

def checksum(data):
    return (~sum(data)) & 0xFF

# ==================================================================
# Smart Servo
# ==================================================================
def smart_write_position(self, position, time_ms=300):

    pos_low = position & 0xFF
    pos_high = (position >> 8) & 0xFF

    time_low = time_ms & 0xFF
    time_high = (time_ms >> 8) & 0xFF

    packet = [
        0xFF,
        0xFF,
        self.id,
        7,
        3,
        0x2A,
        pos_low,
        pos_high,
        time_low,
        time_high
    ]

    packet.append(checksum(packet[2:]))

    self.ser.write(bytes(packet))


def smart_read(self, register, size):

    packet = [
        0xFF,
        0xFF,
        self.id,
        4,
        0x02,
        register,
        size
    ]

    packet.append(checksum(packet[2:]))

    self.ser.reset_input_buffer()

    self.ser.write(bytes(packet))

    response_length = 6 + size
    response = self.ser.read(response_length)

    if len(response) != response_length:
        return None

    if size == 1:
        return response[5]

    elif size == 2:
        low = response[5]
        high = response[6]

        return low | (high << 8)

    return None

# ====================================================================
# Hobby Servo
# ====================================================================
def hobby_write_position(self, position):

    angle = int(position)

    command = f"{self.id}:{angle}\n"

    self.ser.write(command.encode())

# ====================================================================
# Joints
# ====================================================================

pincher = Servo(
    1,
    "smart"
)

base = Servo(
    0,
    "hobby"
)

arm = Servo(
    1,
    "hobby"
)

elbow = Servo(
    2,
    "hobby"
)

forearm = Servo(
    3,
    "hobby"
)

wrist = Servo(
    4,
    "hobby"
)

hand = Servo(
    5,
    "hobby"
)