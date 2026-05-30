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

# Smart servo (Pincher — Feetech STS)
SMART_PORT = "/dev/ttyACM0"
SMART_BAUD = 1000000
SMART_ID   = 1

PINCHER_MIN = 1000   # fully open
PINCHER_MAX = 2200   # closed + grip force
PINCHER_DEF = 1500

TEMP_WARN = 55   # °C — yellow warning
TEMP_CRIT = 65   # °C — red critical (stop using it!)

try:
    smart_ser = serial.Serial(SMART_PORT, SMART_BAUD, timeout=0.3)
except Exception as e:
    print(f"[WARN] Smart servo not connected: {e}")
    smart_ser = None


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
    ser.write(f"{servo_id}:{angle_cmd:.2f}\n".encode())


# =========================================================
# Pincher (smart servo)
# =========================================================

def _smart_checksum(data):
    return (~sum(data)) & 0xFF

def pincher_write(position, time_ms=300):
    if not smart_ser:
        return
    position = max(PINCHER_MIN, min(PINCHER_MAX, int(position)))
    pl, ph = position & 0xFF, (position >> 8) & 0xFF
    tl, th = time_ms & 0xFF, (time_ms >> 8) & 0xFF
    pkt = [0xFF, 0xFF, SMART_ID, 7, 3, 0x2A, pl, ph, tl, th]
    pkt.append(_smart_checksum(pkt[2:]))
    smart_ser.write(bytes(pkt))

def pincher_read_temp():
    if not smart_ser:
        return None
    try:
        pkt = [0xFF, 0xFF, SMART_ID, 4, 0x02, 0x3F, 1]
        pkt.append(_smart_checksum(pkt[2:]))
        smart_ser.reset_input_buffer()
        smart_ser.write(bytes(pkt))
        resp = smart_ser.read(7)
        return resp[5] if len(resp) == 7 else None
    except Exception:
        return None


# =========================================================
# Temperature history (background polling every 2 s)
# =========================================================

_temp_history = []   # list of {"t": epoch_s, "c": celsius}
_temp_lock    = threading.Lock()
_pincher_pos  = PINCHER_DEF

def _temp_poller():
    while True:
        temp = pincher_read_temp()
        if temp is not None:
            with _temp_lock:
                _temp_history.append({"t": time.time(), "c": int(temp)})
                if len(_temp_history) > 60:
                    _temp_history.pop(0)
        time.sleep(2)

_tp = threading.Thread(target=_temp_poller, daemon=True)
_tp.start()


# =========================================================
# Gradual movement
# =========================================================

# Tracks where each servo currently is (in servo cmd degrees)
current_cmds = {name: float(SERVO_DEFS[name].default_pos) for name in SERVO_NAMES}

_move_thread = None
_stop_event  = threading.Event()
_cmd_lock    = threading.Lock()

def move_to(target_cmds, speed_pct=50.0):
    """Move all servos to target_cmds, synchronized so they all finish at the same time.
    speed_pct: 1-100, where 100 is fastest (~10ms per step)."""
    global _move_thread

    _stop_event.set()
    if _move_thread and _move_thread.is_alive():
        _move_thread.join(timeout=1.0)
    _stop_event.clear()

    with _cmd_lock:
        deltas = {name: target - current_cmds[name] for name, target in target_cmds.items()}

    max_delta = max((abs(d) for d in deltas.values()), default=0)
    if max_delta < 0.5:
        return

    n_steps    = max(1, round(max_delta))
    step_sizes = {name: d / n_steps for name, d in deltas.items()}
    step_delay = 0.01 / max(0.01, speed_pct / 100.0)

    def _run():
        for _ in range(n_steps):
            if _stop_event.is_set():
                break
            with _cmd_lock:
                for name, step in step_sizes.items():
                    current_cmds[name] += step
                    send_servo(SERVO_IDS[name], current_cmds[name])
            time.sleep(step_delay)
        if not _stop_event.is_set():
            with _cmd_lock:
                for name, tgt in target_cmds.items():
                    current_cmds[name] = tgt
                    send_servo(SERVO_IDS[name], tgt)

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

  /* Pincher */
  #pincher-section { margin-top: 32px; }
  #pincher-btns { display: flex; gap: 10px; margin-top: 10px; }
  .p-btn { flex: 1; padding: 10px; border-radius: 5px; border: 1px solid #444;
           background: #1e1e1e; color: #ddd; font-size: 13px; cursor: pointer; }
  .p-btn:hover { background: #2a2a2a; }

  /* Temperature */
  #temp-section { margin-top: 32px; }
  #temp-value { font-size: 42px; font-weight: bold; font-family: monospace;
                text-align: center; padding: 10px 0; }
  .temp-ok   { color: #3d3; }
  .temp-warn { color: #fa4; }
  .temp-crit { color: #f33; }
  #temp-alert { display: none; background: #4a0000; color: #f66; border: 1px solid #f33;
                border-radius: 6px; padding: 12px; text-align: center;
                font-weight: bold; margin-bottom: 10px; }
  #temp-canvas { width: 100%; height: 120px; background: #1a1a1a;
                 border: 1px solid #2a2a2a; border-radius: 4px; display: block; }
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
  <span class="form-label">Speed (%)</span>
  <input type="number" id="Speed" value="50" min="1" max="100" step="1">
  <input type="range" id="Speed-s" min="1" max="100" step="1" value="50">
  <span class="limits">1 – 100%</span>
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

<!-- ======================================================= -->
<!-- Pincher                                                  -->
<!-- ======================================================= -->
<div id="pincher-section">
  <h2>Pincher</h2>
  <div class="form-row">
    <span class="form-label">Position</span>
    <input type="number" id="Pincher" value="1500" min="1000" max="2200" step="10">
    <input type="range" id="Pincher-s" min="1000" max="2200" step="10" value="1500">
    <span class="limits">1000 – 2200</span>
  </div>
  <div id="pincher-btns">
    <button class="p-btn" onclick="setPincher(1000)">Open</button>
    <button class="p-btn" onclick="setPincher(2050)">Close</button>
    <button class="p-btn" onclick="setPincher(2200)">Grip</button>
  </div>
</div>

<!-- ======================================================= -->
<!-- Temperature                                              -->
<!-- ======================================================= -->
<div id="temp-section">
  <h2>Pincher Temperature</h2>
  <div id="temp-alert">⚠ TEMPERATURE CRITICAL — let the servo cool down!</div>
  <div id="temp-value" class="temp-ok">— °C</div>
  <canvas id="temp-canvas" width="720" height="120"></canvas>
  <div style="display:flex;justify-content:space-between;font-size:10px;color:#444;margin-top:4px;">
    <span>2 min ago</span>
    <span style="color:#fa4">warn 55°C</span>
    <span style="color:#f33">crit 65°C</span>
    <span>now</span>
  </div>
</div>

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

// ---- Pincher ----
const pNum = document.getElementById("Pincher");
const pSld = document.getElementById("Pincher-s");
pNum.addEventListener("input", () => { pSld.value = pNum.value; sendPincher(); });
pSld.addEventListener("input", () => { pNum.value = pSld.value; sendPincher(); });

function setPincher(pos) { pNum.value = pos; pSld.value = pos; sendPincher(); }

function sendPincher() {
  fetch("/pincher", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ position: parseInt(pNum.value) })
  });
}

// ---- Temperature graph ----
const TEMP_WARN = 55, TEMP_CRIT = 65;
let tempHistory = [];

function pollTemp() {
  fetch("/temperature")
    .then(r => r.json())
    .then(d => {
      tempHistory = d.history || [];
      const cur = d.current;
      const el  = document.getElementById("temp-value");
      const alrt = document.getElementById("temp-alert");
      if (cur === null) {
        el.textContent = "— °C";
        el.className = "temp-ok";
      } else {
        el.textContent = cur + " °C";
        el.className = cur >= TEMP_CRIT ? "temp-crit" : cur >= TEMP_WARN ? "temp-warn" : "temp-ok";
        alrt.style.display = cur >= TEMP_CRIT ? "block" : "none";
      }
      drawGraph(tempHistory);
    })
    .catch(() => {});
}

function drawGraph(history) {
  const canvas = document.getElementById("temp-canvas");
  const ctx    = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  // Temperature range 20–80°C displayed
  const T_MIN = 20, T_MAX = 80;
  const yOf = t => H - ((t - T_MIN) / (T_MAX - T_MIN)) * H;

  // Warn / crit bands
  ctx.fillStyle = "rgba(250,160,0,0.08)";
  ctx.fillRect(0, yOf(TEMP_CRIT), W, yOf(TEMP_WARN) - yOf(TEMP_CRIT));
  ctx.fillStyle = "rgba(220,0,0,0.12)";
  ctx.fillRect(0, 0, W, yOf(TEMP_CRIT));

  // Gridlines
  [30, 40, 50, 55, 65, 70].forEach(t => {
    ctx.strokeStyle = t === TEMP_WARN ? "#fa4" : t === TEMP_CRIT ? "#f33" : "#2a2a2a";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, yOf(t)); ctx.lineTo(W, yOf(t)); ctx.stroke();
  });

  if (history.length < 2) return;

  const now    = history[history.length - 1].t;
  const tStart = now - 120;  // 2 minutes window
  const xOf = t => ((t - tStart) / 120) * W;

  ctx.beginPath();
  ctx.lineWidth = 2;
  history.forEach((p, i) => {
    const x = xOf(p.t), y = yOf(p.c);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  const last = history[history.length - 1].c;
  ctx.strokeStyle = last >= TEMP_CRIT ? "#f33" : last >= TEMP_WARN ? "#fa4" : "#3d3";
  ctx.stroke();
}

setInterval(pollTemp, 2000);
pollTemp();

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
        target_roll  = float(data.get("Roll", SERVO_DEFS["Hand"].position))
        speed_pct    = float(data.get("Speed", 50))

        ik = inverse_kinematics(target_pos, target_pitch, target_yaw, target_roll=target_roll)

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
            move_to({name: s["cmd"] for name, s in servos.items()}, speed_pct)

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


@app.route("/pincher", methods=["POST"])
def pincher_route():
    try:
        pos = int(request.json.get("position", PINCHER_DEF))
        pincher_write(pos)
        global _pincher_pos
        _pincher_pos = pos
        return jsonify({"position": pos})
    except Exception as e:
        return jsonify({"error": str(e)}), 200


@app.route("/temperature", methods=["GET"])
def temperature_route():
    with _temp_lock:
        history = list(_temp_history)
    current = history[-1]["c"] if history else None
    return jsonify({"current": current, "warn": TEMP_WARN, "crit": TEMP_CRIT, "history": history})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
