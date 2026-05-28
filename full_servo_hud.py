from flask import Flask, render_template_string, request, jsonify
import serial
import threading
import time
import math

from robotic_arm_logic import inverse_kinematics, SERVOS as SERVO_DEFS


# =========================================================
# Serial
# =========================================================

HOBBY_PORT = "/dev/ttyUSB0"
HOBBY_BAUD = 115200

ser = serial.Serial(HOBBY_PORT, HOBBY_BAUD, timeout=1)
time.sleep(2)


# =========================================================
# Config
# =========================================================

app = Flask(__name__)

SERVO_NAMES = ["Base", "Arm", "Elbow", "Forearm", "Wrist", "Hand"]
SERVO_IDS   = {"Base": 0, "Arm": 1, "Elbow": 2, "Forearm": 3, "Wrist": 4, "Hand": 5}


FLIPPED = {"Arm"}

def kin_to_cmd(sdef, name, kin_deg):
    cmd = kin_deg - sdef.conversion_angle + sdef.default_pos
    if name in FLIPPED:
        cmd = 2 * sdef.default_pos - cmd
    return cmd


def send_servo(servo_id, angle_cmd):
    ser.write(f"{servo_id}:{int(angle_cmd)}\n".encode())


# =========================================================
# Gradual movement
# =========================================================

# Tracks where each servo currently is (in servo cmd degrees)
current_cmds = {name: float(SERVO_DEFS[name].default_pos) for name in SERVO_NAMES}

_move_thread = None
_stop_event  = threading.Event()
_cmd_lock    = threading.Lock()

STEP_DEG = 1  # degrees per step (resolution of each increment)


def move_to(target_cmds, speed_deg_s=60.0):
    """Spin up a background thread that steps every servo toward target_cmds."""
    global _move_thread

    _stop_event.set()
    if _move_thread and _move_thread.is_alive():
        _move_thread.join(timeout=1.0)
    _stop_event.clear()

    delay = STEP_DEG / max(speed_deg_s, 1.0)

    def _run():
        while not _stop_event.is_set():
            all_done = True
            with _cmd_lock:
                for name, tgt in target_cmds.items():
                    curr = current_cmds[name]
                    diff = tgt - curr
                    if abs(diff) <= STEP_DEG:
                        current_cmds[name] = tgt
                        step_val = tgt
                    else:
                        step_val = curr + STEP_DEG * (1 if diff > 0 else -1)
                        current_cmds[name] = step_val
                        all_done = False
                    send_servo(SERVO_IDS[name], step_val)
            if all_done:
                break
            time.sleep(delay)

    _move_thread = threading.Thread(target=_run, daemon=True)
    _move_thread.start()


# =========================================================
# HTML
# =========================================================

HTML = """
<!DOCTYPE html>
<html>
<head>
<title>Robot Arm — IK Controller</title>
<style>
  * { box-sizing: border-box; }
  body {
    font-family: Arial, sans-serif;
    background: #111;
    color: #ddd;
    padding: 30px;
    max-width: 780px;
  }
  h1 { color: #fff; margin-bottom: 4px; }
  .subtitle { color: #555; font-size: 13px; margin-bottom: 28px; }
  h2 {
    color: #888;
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 1px;
    margin: 28px 0 12px;
    border-bottom: 1px solid #222;
    padding-bottom: 6px;
  }

  .form-row {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-bottom: 12px;
  }
  .form-label {
    width: 72px;
    font-size: 13px;
    color: #aaa;
    flex-shrink: 0;
  }
  input[type=number] {
    width: 90px;
    padding: 6px 8px;
    background: #1e1e1e;
    border: 1px solid #333;
    border-radius: 4px;
    color: #fff;
    font-size: 13px;
    font-family: monospace;
    flex-shrink: 0;
  }
  input[type=number]:focus { outline: none; border-color: #39f; }
  input[type=range] {
    flex: 1;
    accent-color: #39f;
    cursor: pointer;
    min-width: 0;
  }
  .limits { font-size: 11px; color: #444; flex-shrink: 0; }

  #send-btn {
    margin-top: 20px;
    padding: 10px 32px;
    background: #1a3a5a;
    border: 1px solid #39f;
    border-radius: 5px;
    color: #7cf;
    font-size: 14px;
    cursor: pointer;
    letter-spacing: 0.5px;
  }
  #send-btn:hover { background: #1e4a70; }
  #send-btn:active { background: #0d2a45; }

  #status {
    margin: 22px 0 6px;
    padding: 12px 18px;
    border-radius: 6px;
    font-size: 14px;
    font-weight: bold;
    text-align: center;
    transition: background 0.15s;
  }
  .idle      { background: #1e1e1e; color: #555; }
  .in-reach  { background: #0d2e0d; color: #3d3; border: 1px solid #1a4a1a; }
  .out-reach { background: #2e0d0d; color: #d33; border: 1px solid #4a1a1a; }
  .error     { background: #2e200d; color: #fa0; border: 1px solid #4a3a0d; }

  table { width: 100%; border-collapse: collapse; margin-top: 10px; font-size: 12px; }
  th {
    color: #555; font-size: 10px; text-transform: uppercase;
    letter-spacing: 0.5px; padding: 6px 8px; text-align: left;
    border-bottom: 1px solid #222;
  }
  td { padding: 7px 8px; font-family: monospace; border-bottom: 1px solid #1a1a1a; }
  td:first-child { font-family: Arial; color: #aaa; }
  .ok  { color: #3d3; }
  .bad { color: #d33; font-weight: bold; }
  .dim { color: #444; }
</style>
</head>
<body>

<h1>Robot Arm — IK Controller</h1>
<div class="subtitle">Inverse kinematics &nbsp;·&nbsp; port 8080</div>

<h2>Target Position</h2>
<div class="form-row">
  <span class="form-label">X (mm)</span>
  <input type="number" id="X" value="300" step="5">
  <input type="range" id="X-s" min="-1000" max="1000" step="5" value="300">
  <span class="limits">±1000</span>
</div>
<div class="form-row">
  <span class="form-label">Y (mm)</span>
  <input type="number" id="Y" value="0" step="5">
  <input type="range" id="Y-s" min="-1000" max="1000" step="5" value="0">
  <span class="limits">±1000</span>
</div>
<div class="form-row">
  <span class="form-label">Z (mm)</span>
  <input type="number" id="Z" value="300" step="5">
  <input type="range" id="Z-s" min="0" max="1000" step="5" value="300">
  <span class="limits">0 – 1000</span>
</div>

<h2>Target Direction</h2>
<div class="form-row">
  <span class="form-label">Pitch (°)</span>
  <input type="number" id="Pitch" value="-20" step="1">
  <input type="range" id="Pitch-s" min="-90" max="90" step="1" value="-20">
  <span class="limits">±90°</span>
</div>
<div class="form-row">
  <span class="form-label">Yaw (°)</span>
  <input type="number" id="Yaw" value="0" step="1">
  <input type="range" id="Yaw-s" min="-180" max="180" step="1" value="0">
  <span class="limits">±180°</span>
</div>

<h2>Movement</h2>
<div class="form-row">
  <span class="form-label">Speed (°/s)</span>
  <input type="number" id="Speed" value="60" min="5" max="300" step="5">
  <input type="range" id="Speed-s" min="5" max="300" step="5" value="60">
  <span class="limits">5 – 300</span>
</div>

<button id="send-btn" onclick="sendIK()">Send to Robot</button>

<div id="status" class="idle">—</div>

<h2>Servo Angles</h2>
<table>
  <thead>
    <tr>
      <th>Servo</th><th>Kinematic</th><th>Command</th><th>Limits</th><th></th>
    </tr>
  </thead>
  <tbody id="servo-table">
    <tr><td colspan="5" class="dim" style="text-align:center;padding:14px">
      Move a slider or press Send
    </td></tr>
  </tbody>
</table>

<script>
const FIELDS = ["X", "Y", "Z", "Pitch", "Yaw", "Speed"];
let debounce = null;

function val(id) { return parseFloat(document.getElementById(id).value) || 0; }

function clampToSlider(id, v) {
  const s = document.getElementById(id + "-s");
  return Math.max(parseFloat(s.min), Math.min(parseFloat(s.max), v));
}

// Wire each field: number ↔ slider, both trigger debounced send
FIELDS.forEach(id => {
  const num = document.getElementById(id);
  const sld = document.getElementById(id + "-s");

  num.addEventListener("input", () => {
    sld.value = clampToSlider(id, parseFloat(num.value) || 0);
    scheduleSend();
  });

  sld.addEventListener("input", () => {
    num.value = sld.value;
    scheduleSend();
  });
});

function scheduleSend() {
  clearTimeout(debounce);
  debounce = setTimeout(sendIK, 80);
}

function sendIK() {
  clearTimeout(debounce);
  const btn = document.getElementById("send-btn");
  btn.disabled = true;

  fetch("/ik", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      X: val("X"), Y: val("Y"), Z: val("Z"),
      Pitch: val("Pitch"), Yaw: val("Yaw"),
      Speed: val("Speed")
    })
  })
  .then(r => r.json())
  .then(data => { updateDisplay(data); btn.disabled = false; })
  .catch(() => {
    document.getElementById("status").className = "error";
    document.getElementById("status").textContent = "Network error";
    btn.disabled = false;
  });
}

document.addEventListener("keydown", e => { if (e.key === "Enter") sendIK(); });

function updateDisplay(data) {
  const statusEl = document.getElementById("status");
  if (data.status === "error") {
    statusEl.className = "error";
    statusEl.textContent = "Error: " + data.message;
    return;
  }
  statusEl.className = data.status === "in_reach" ? "in-reach" : "out-reach";
  statusEl.textContent = data.status === "in_reach"
    ? "IN REACH — moving to position"
    : "OUT OF REACH: " + data.out_of_range.join(", ");

  const tbody = document.getElementById("servo-table");
  tbody.innerHTML = "";
  for (const [name, s] of Object.entries(data.servos)) {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${name}</td>
      <td>${s.kin >= 0 ? "+" : ""}${s.kin.toFixed(1)}°</td>
      <td>${s.cmd}°</td>
      <td class="dim">${s.min} – ${s.max}</td>
      <td class="${s.ok ? "ok" : "bad"}">${s.ok ? "✓" : "✗"}</td>
    `;
    tbody.appendChild(tr);
  }
}
</script>

</body>
</html>
"""


# =========================================================
# Routes
# =========================================================

@app.route("/")
def home():
    return render_template_string(HTML)


@app.route("/ik", methods=["POST"])
def ik_route():
    try:
        data = request.json
        target_pos   = {"x": float(data["X"]), "y": float(data["Y"]), "z": float(data["Z"])}
        target_pitch = math.radians(float(data["Pitch"]))
        target_yaw   = math.radians(float(data["Yaw"]))
        speed        = float(data.get("Speed", 60))

        ik = inverse_kinematics(target_pos, target_pitch, target_yaw)

        out_of_range = []
        servos = {}

        for name in SERVO_NAMES:
            if name not in ik:
                continue
            sdef = SERVO_DEFS[name]
            kin  = ik[name]
            cmd  = kin_to_cmd(sdef, name, kin)
            ok   = sdef.limits[0] <= cmd <= sdef.limits[1]
            servos[name] = {
                "kin": round(float(kin), 1),
                "cmd": round(float(cmd)),
                "min": sdef.limits[0],
                "max": sdef.limits[1],
                "ok":  ok,
            }
            if not ok:
                out_of_range.append(name)

        if not out_of_range:
            move_to({name: s["cmd"] for name, s in servos.items()}, speed)

        response = {
            "status":       "in_reach" if not out_of_range else "out_of_reach",
            "out_of_range": out_of_range,
            "servos":       servos,
            "target":       data,
        }
        app.config["last_state"] = response
        return jsonify(response)

    except Exception as e:
        import traceback
        return jsonify({"status": "error", "message": str(e), "trace": traceback.format_exc()}), 200


@app.route("/state", methods=["GET"])
def state():
    return jsonify(app.config.get("last_state", {}))


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
