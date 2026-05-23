import tkinter as tk
import serial

# Waveshare adapter port
PORT = "/dev/ttyACM0"

# Servo settings
BAUD = 1000000
SERVO_ID = 1

# Open serial connection
ser = serial.Serial(PORT, BAUD, timeout=1)

# Full servo range
MIN_POS = 0
MAX_POS = 4095


def checksum(data):
    return (~sum(data)) & 0xFF


def write_position(position, time_ms=300):

    pos_low = position & 0xFF
    pos_high = (position >> 8) & 0xFF

    time_low = time_ms & 0xFF
    time_high = (time_ms >> 8) & 0xFF

    packet = [
        0xFF,
        0xFF,
        SERVO_ID,
        7,
        3,
        0x2A,
        pos_low,
        pos_high,
        time_low,
        time_high
    ]

    packet.append(checksum(packet[2:]))

    ser.write(bytes(packet))


def slider_changed(val):

    angle = int(float(val))

    # Convert 0-360° to servo position
    target = int((angle / 360.0) * MAX_POS)

    print("Angle:", angle, " Position:", target)

    write_position(target)


# GUI
root = tk.Tk()
root.title("Feetech STS Servo - Full 360° Control")

label = tk.Label(
    root,
    text="Full 360° Servo Control",
    font=("Arial", 14)
)

label.pack(pady=10)

slider = tk.Scale(
    root,
    from_=0,
    to=360,
    orient="horizontal",
    length=700,
    resolution=1,
    command=slider_changed,
    label="Angle"
)

slider.set(180)

slider.pack(padx=20, pady=20)

root.mainloop()