import tkinter as tk
import threading
import serial
from flask import Flask, request, jsonify

# =========================================================
# Serial
# =========================================================

PORT     = "/dev/ttyACM0"
BAUD     = 1000000
SERVO_ID = 1

ser = serial.Serial(PORT, BAUD, timeout=1)

# Extend MIN below 0 until the gripper fully opens.
# 4096 ticks = 360°, so -1000 ≈ 88° past the zero point.
MIN_POS = -1000
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

    # Python's bitwise ops handle negative values correctly (two's complement)
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

HTML = f"""
<!DOCTYPE html>
<html>
<head>
<title>Pincher Servo</title>
<style>
  body {{ font-family: Arial; background: #111; color: #ddd; padding: 30px; max-width: 500px; }}
  h1   {{ color: #fff; }}
  .sub {{ color: #555; font-size: 12px; margin-bottom: 20px; }}
  .row {{ display: flex; align-items: center; gap: 14px; margin-top: 20px; }}
  input[type=range]  {{ flex: 1; accent-color: #39f; }}
  input[type=number] {{ width: 90px; padding: 6px 8px; background: #1e1e1e;
                        border: 1px solid #333; border-radius: 4px; color: #fff;
                        font-size: 13px; font-family: monospace; }}
  .lbl {{ width: 80px; color: #aaa; font-size: 13px; }}
  #send {{ margin-top: 20px; padding: 10px 28px; background: #1a3a5a;
           border: 1px solid #39f; border-radius: 5px; color: #7cf;
           font-size: 14px; cursor: pointer; }}
  #send:hover {{ background: #1e4a70; }}
  #status {{ margin-top: 14px; color: #888; font-size: 13px; }}
</style>
</head>
<body>
<h1>Pincher Servo</h1>
<div class="sub">Range: {MIN_POS} – {MAX_POS} &nbsp;·&nbsp; 4096 ticks = 360°</div>

<div class="row">
  <span class="lbl">Position</span>
  <input type="range" id="pos-s" min="{MIN_POS}" max="{MAX_POS}" step="1" value="2042">
  <input type="number" id="pos" value="2042" min="{MIN_POS}" max="{MAX_POS}" step="1">
</div>

<button id="send" onclick="send()">Send</button>
<div id="status">—</div>

<script>
const sld = document.getElementById("pos-s");
const num = document.getElementById("pos");
let deb = null;

sld.addEventListener("input", () => {{ num.value = sld.value; schedule(); }});
num.addEventListener("input", () => {{ sld.value = Math.min({MAX_POS}, Math.max({MIN_POS}, +num.value || 0)); schedule(); }});

function schedule() {{ clearTimeout(deb); deb = setTimeout(send, 80); }}

function send() {{
  const pos = parseInt(num.value) || 0;
  fetch("/pincher", {{
    method: "POST",
    headers: {{ "Content-Type": "application/json" }},
    body: JSON.stringify({{ position: pos }})
  }})
  .then(r => r.json())
  .then(d => {{ document.getElementById("status").textContent = "sent: " + d.position; }})
  .catch(() => {{ document.getElementById("status").textContent = "error"; }});
}}
</script>
</body>
</html>
"""


@app.route("/")
def home():
    return HTML


@app.route("/pincher", methods=["POST"])
def pincher():
    data = request.json
    pos  = int(data.get("position", current_position))
    write_position(pos)
    if slider_ref:
        slider_ref.set(pos)
    return jsonify({"position": current_position})


@app.route("/state", methods=["GET"])
def state():
    return jsonify({"position": current_position})


# =========================================================
# Tkinter GUI
# =========================================================

slider_ref = None


def slider_changed(val):
    pos = int(float(val))
    print(f"Position: {pos}")
    write_position(pos)


def start_gui():
    global slider_ref

    root = tk.Tk()
    root.title("Feetech STS Servo")

    tk.Label(root, text="Pincher Servo", font=("Arial", 14)).pack(pady=10)

    slider_ref = tk.Scale(
        root, from_=MIN_POS, to=MAX_POS, orient="horizontal",
        length=700, resolution=1, command=slider_changed, label="Position"
    )
    slider_ref.set(2042)
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
