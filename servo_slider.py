import tkinter as tk
import threading
import serial
from flask import Flask, request, jsonify, render_template_string

# =========================================================
# Serial
# =========================================================

PORT     = "/dev/ttyACM0"
BAUD     = 1000000
SERVO_ID = 1

ser = serial.Serial(PORT, BAUD, timeout=1)

MIN_POS = 0
MAX_POS = 4095

current_position = 2042


# =========================================================
# Servo protocol
# =========================================================

def checksum(data):
    return (~sum(data)) & 0xFF


def write_position(position, time_ms=300):
    global current_position
    position = max(MIN_POS, min(MAX_POS, int(position)))
    current_position = position

    pos_low  = position & 0xFF
    pos_high = (position >> 8) & 0xFF
    time_low  = time_ms & 0xFF
    time_high = (time_ms >> 8) & 0xFF

    packet = [0xFF, 0xFF, SERVO_ID, 7, 3, 0x2A,
               pos_low, pos_high, time_low, time_high]
    packet.append(checksum(packet[2:]))
    ser.write(bytes(packet))


# =========================================================
# Flask — network interface
# =========================================================

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
<title>Pincher Servo</title>
<style>
  body { font-family: Arial; background: #111; color: #ddd; padding: 30px; max-width: 500px; }
  h1   { color: #fff; }
  .row { display: flex; align-items: center; gap: 14px; margin-top: 20px; }
  input[type=range]  { flex: 1; accent-color: #39f; }
  input[type=number] { width: 90px; padding: 6px 8px; background: #1e1e1e;
                       border: 1px solid #333; border-radius: 4px; color: #fff;
                       font-size: 13px; font-family: monospace; }
  .lbl { width: 80px; color: #aaa; font-size: 13px; }
  #send { margin-top: 20px; padding: 10px 28px; background: #1a3a5a;
          border: 1px solid #39f; border-radius: 5px; color: #7cf;
          font-size: 14px; cursor: pointer; }
  #send:hover { background: #1e4a70; }
  #status { margin-top: 14px; color: #888; font-size: 13px; }
</style>
</head>
<body>
<h1>Pincher Servo</h1>

<div class="row">
  <span class="lbl">Angle (°)</span>
  <input type="range" id="angle-s" min="0" max="360" step="1" value="180">
  <input type="number" id="angle" value="180" min="0" max="360" step="1">
</div>

<button id="send" onclick="send()">Send</button>
<div id="status">—</div>

<script>
const sld = document.getElementById("angle-s");
const num = document.getElementById("angle");
let deb = null;

sld.addEventListener("input", () => { num.value = sld.value; schedule(); });
num.addEventListener("input", () => { sld.value = Math.min(360, Math.max(0, num.value)); schedule(); });

function schedule() { clearTimeout(deb); deb = setTimeout(send, 80); }

function send() {
  const angle = parseFloat(num.value) || 0;
  fetch("/pincher", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ angle })
  })
  .then(r => r.json())
  .then(d => { document.getElementById("status").textContent = "position: " + d.position; })
  .catch(() => { document.getElementById("status").textContent = "error"; });
}
</script>
</body>
</html>
"""


@app.route("/")
def home():
    return render_template_string(HTML)


@app.route("/pincher", methods=["POST"])
def pincher():
    data  = request.json
    angle = float(data.get("angle", 180))
    angle = max(0.0, min(360.0, angle))
    pos   = int((angle / 360.0) * MAX_POS)
    write_position(pos)
    if slider_ref:
        slider_ref.set(angle)
    return jsonify({"angle": angle, "position": pos})


@app.route("/state", methods=["GET"])
def state():
    return jsonify({"position": current_position,
                    "angle": round(current_position / MAX_POS * 360, 2)})


# =========================================================
# Tkinter GUI
# =========================================================

slider_ref = None


def slider_changed(val):
    angle = int(float(val))
    pos   = int((angle / 360.0) * MAX_POS)
    print(f"Angle: {angle}°  Position: {pos}")
    write_position(pos)


def start_gui():
    global slider_ref

    root = tk.Tk()
    root.title("Feetech STS Servo — 360° Control")

    tk.Label(root, text="Full 360° Servo Control", font=("Arial", 14)).pack(pady=10)

    slider_ref = tk.Scale(
        root, from_=0, to=360, orient="horizontal",
        length=700, resolution=1, command=slider_changed, label="Angle"
    )
    slider_ref.set(180)
    slider_ref.pack(padx=20, pady=20)

    root.mainloop()


# =========================================================
# Entry point
# =========================================================

if __name__ == "__main__":
    import os
    has_display = bool(os.environ.get("DISPLAY"))

    if has_display:
        flask_thread = threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=8081, threaded=True),
            daemon=True
        )
        flask_thread.start()
        print("Pincher web interface running on port 8081")
        start_gui()
    else:
        print("No display — running web interface only on port 8081")
        app.run(host="0.0.0.0", port=8081, threaded=True)
