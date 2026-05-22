import tkinter as tk
import serial
import time

PORT = "/dev/ttyS3"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(3)  # allow Arduino reset

def send_angle(index, value):
    angle = int(float(value))
    ser.write(f"{index}:{angle}\n".encode())

def on_close():
    ser.close()
    root.destroy()

root = tk.Tk()
root.title("Servo Controller")
root.geometry("500x500")

for i in range(6):
    label = tk.Label(root, text=f"Servo {i} (Channel {i*2})")
    label.pack()

    slider = tk.Scale(
        root,
        from_=0,
        to=270,
        orient="horizontal",
        length=400,
        command=lambda val, idx=i: send_angle(idx, val)
    )
    slider.set(135)
    slider.pack(pady=5)

# Initialize all servos to center
for i in range(6):
    send_angle(i, 135)

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()