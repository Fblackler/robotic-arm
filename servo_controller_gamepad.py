import pygame
import serial
import time

# =========================================================
# SERIAL CONFIG
# =========================================================

# Arduino (normal servos)
ARDUINO_PORT = "/dev/tty.usbserial-10"
ARDUINO_BAUD = 115200

# Waveshare smart servo adapter
SMART_PORT = "/dev/cu.usbmodem5B141156511"
SMART_BAUD = 1000000

# =========================================================
# NORMAL SERVOS
# =========================================================

SERVO_CONFIG = {

    # Servo 0
    0: {
        "type": "axis",
        "input": 2,   # right stick X
    },

    # Servo 1
    1: {
        "type": "axis",
        "input": 3,   # right stick Y
    },

    # Servo 2
    2: {
        "type": "axis",
        "input": 0,   # left stick X
    },

    # Servo 3
    3: {
        "type": "axis",
        "input": 1,   # left stick Y
    },

    # Servo 4
    4: {
        "type": "button",
        "negative": 9,    # LB
        "positive": 10,   # RB
    },
}

# =========================================================
# SMART SERVO
# =========================================================

SMART_SERVO = {

    "id": 1,

    # Center-ish
    "position": 2048,

    # Speed of trigger movement
    "step": 40,

    # Trigger bindings
    "negative_trigger": 4,   # LT
    "positive_trigger": 5,   # RT
}

# =========================================================

DEADZONE = 0.1
SPEED = 3

# =========================================================
# SERIAL CONNECTIONS
# =========================================================

arduino_ser = serial.Serial(
    ARDUINO_PORT,
    ARDUINO_BAUD,
    timeout=1
)

smart_ser = serial.Serial(
    SMART_PORT,
    SMART_BAUD,
    timeout=1
)

time.sleep(3)

# =========================================================
# PYGAME
# =========================================================

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller connected:", joystick.get_name())

# =========================================================
# NORMAL SERVO STATE
# =========================================================

NUM_SERVOS = len(SERVO_CONFIG)

servo_angles = [135] * NUM_SERVOS

# =========================================================
# SMART SERVO FUNCTIONS
# =========================================================

def checksum(data):
    return (~sum(data)) & 0xFF


def send_smart_servo(position, time_ms=100):

    SMART_SERVO["position"] = max(
        0,
        min(4095, position)
    )

    pos_low = SMART_SERVO["position"] & 0xFF
    pos_high = (SMART_SERVO["position"] >> 8) & 0xFF

    time_low = time_ms & 0xFF
    time_high = (time_ms >> 8) & 0xFF

    packet = [
        0xFF,
        0xFF,
        SMART_SERVO["id"],
        7,
        3,
        0x2A,
        pos_low,
        pos_high,
        time_low,
        time_high
    ]

    packet.append(checksum(packet[2:]))

    smart_ser.write(bytes(packet))

# =========================================================
# NORMAL SERVO FUNCTION
# =========================================================

def send_angle(index, angle):
    arduino_ser.write(f"{index}:{angle}\n".encode())

# =========================================================
# INIT NORMAL SERVOS
# =========================================================

for i in range(NUM_SERVOS):
    send_angle(i, 135)

# =========================================================
# MAIN LOOP
# =========================================================

try:

    while True:

        pygame.event.pump()

        # =================================================
        # NORMAL SERVOS
        # =================================================

        for channel, config in SERVO_CONFIG.items():

            # =============================================
            # AXIS CONTROL
            # =============================================

            if config["type"] == "axis":

                axis_id = config["input"]

                val = joystick.get_axis(axis_id)

                if abs(val) < DEADZONE:
                    continue

                delta = int(val * SPEED)

                servo_angles[channel] += delta

            # =============================================
            # BUTTON CONTROL
            # =============================================

            elif config["type"] == "button":

                negative_pressed = joystick.get_button(
                    config["negative"]
                )

                positive_pressed = joystick.get_button(
                    config["positive"]
                )

                if negative_pressed:
                    servo_angles[channel] -= SPEED

                if positive_pressed:
                    servo_angles[channel] += SPEED

            # =============================================
            # LIMITS
            # =============================================

            servo_angles[channel] = max(
                0,
                min(270, servo_angles[channel])
            )

            send_angle(
                channel,
                servo_angles[channel]
            )

        # =================================================
        # SMART SERVO
        # =================================================

        lt = joystick.get_axis(
            SMART_SERVO["negative_trigger"]
        )

        rt = joystick.get_axis(
            SMART_SERVO["positive_trigger"]
        )

        # macOS trigger normalization
        lt = max(0, lt)
        rt = max(0, rt)

        move = 0

        if lt > 0.05:
            move -= int(
                lt * SMART_SERVO["step"]
            )

        if rt > 0.05:
            move += int(
                rt * SMART_SERVO["step"]
            )

        if move != 0:

            send_smart_servo(
                SMART_SERVO["position"] + move
            )

        # =================================================

        time.sleep(0.02)

except KeyboardInterrupt:

    print("Exiting...")

    arduino_ser.close()
    smart_ser.close()