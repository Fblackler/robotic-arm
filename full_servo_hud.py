from flask import Flask, render_template_string, request, jsonify
import serial
import time
import math

from robotic_arm_logic import inverse_kinematics, SERVOS as SERVO_DEFS


# =========================================================
# Serial
# =========================================================

HOBBY_PORT = "/dev/tty.usbserial-10"
HOBBY_BAUD = 115200

ser = serial.Serial(HOBBY_PORT, HOBBY_BAUD, timeout=1)
time.sleep(2)


# =========================================================
# Config
# =========================================================

app = Flask(__name__)

SERVO_NAMES = ["Base", "Arm", "Elbow", "Forearm", "Wrist", "Hand"]
SERVO_IDS   = {"Base": 0, "Arm": 1, "Elbow": 2, "Forearm": 3, "Wrist": 4, "Hand": 5}


def kin_to_cmd(sdef, kin_deg):
    return kin_deg - sdef.conversion_angle + sdef.default_pos


def send_servo(servo_id, angle_cmd):
    ser.write(f"{servo_id}:{int(angle_cmd)}\n".encode())


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
    max-width: 720px;
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

  .slider-row {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-bottom: 14px;
  }
  .slider-label {
    width: 52px;
    font-size: 13px;
    color: #aaa;
  }
  .slider-bounds {
    font-size: 10px;
    color: #444;
    width: 70px;
    text-align: right;
    white-space: nowrap;
  }
  input[type=range] {
    flex: 1;
    accent-color: #39f;
    cursor: pointer;
  }
  .slider-value {
    width: 64px;
    text-align: right;
    font-size: 13px;
    font-family: monospace;
    color: #fff;
  }

  #status {
    margin: 24px 0 4px;
    padding: 14px 20px;
    border-radius: 6px;
    font-size: 15px;
    font-weight: bold;
    text-align: center;
    letter-spacing: 0.5px;
    transition: background 0.15s;
  }
  .idle      { background: #1e1e1e; color: #555; }
  .in-reach  { background: #0d2e0d; color: #3d3; border: 1px solid #1a4a1a; }
  .out-reach { background: #2e0d0d; color: #d33; border: 1px solid #4a1a1a; }
  .error     { background: #2e200d; color: #fa0; border: 1px solid #4a3a0d; }

  table {
    width: 100%;
    border-collapse: collapse;
    margin-top: 10px;
    font-size: 12px;
  }
  th {
    color: #555;
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    padding: 6px 8px;
    text-align: left;
    border-bottom: 1px solid #222;
  }
  td {
    padding: 7px 8px;
    font-family: monospace;
    border-bottom: 1px solid #1a1a1a;
  }
  td:first-child { font-family: Arial; color: #aaa; }
  .ok  { color: #3d3; }
  .bad { color: #d33; font-weight: bold; }
  .dim { color: #444; }
</style>
</head>
<body>

<h1>Robot Arm — IK Controller</h1>
<div class="subtitle">Inverse kinematics over network &nbsp;·&nbsp; port 8080</div>

<h2>Target Position</h2>
<div id="pos-sliders"></div>

<h2>Target Direction</h2>
<div id="dir-sliders"></div>

<div id="status" class="idle">—</div>

<h2>Servo Angles</h2>
<table>
  <thead>
    <tr>
      <th>Servo</th>
      <th>Kinematic</th>
      <th>Command</th>
      <th>Limits</th>
      <th>Status</th>
    </tr>
  </thead>
  <tbody id="servo-table">
    <tr><td colspan="5" class="dim" style="text-align:center;padding:14px">
      Move a slider to compute IK
    </td></tr>
  </tbody>
</table>

<script>
const SLIDERS = [
  { id: "X",     label: "X",     min: -700, max: 700,  init: 300,  unit: "mm", group: "pos" },
  { id: "Y",     label: "Y",     min: -700, max: 700,  init: 0,    unit: "mm", group: "pos" },
  { id: "Z",     label: "Z",     min: 0,    max: 700,  init: 300,  unit: "mm", group: "pos" },
  { id: "Pitch", label: "Pitch", min: -90,  max: 90,   init: -20,  unit: "°",  group: "dir" },
  { id: "Yaw",   label: "Yaw",   min: -180, max: 180,  init: 0,    unit: "°",  group: "dir" },
];

const vals = {};
let debounce = null;

// Build sliders
SLIDERS.forEach(cfg => {
  vals[cfg.id] = cfg.init;

  const row = document.createElement("div");
  row.className = "slider-row";

  const label = document.createElement("div");
  label.className = "slider-label";
  label.textContent = cfg.label;

  const bounds = document.createElement("div");
  bounds.className = "slider-bounds";
  bounds.textContent = `${cfg.min} – ${cfg.max}${cfg.unit}`;

  const slider = document.createElement("input");
  slider.type  = "range";
  slider.min   = cfg.min;
  slider.max   = cfg.max;
  slider.step  = cfg.unit === "mm" ? 5 : 1;
  slider.value = cfg.init;
  slider.id    = "slider-" + cfg.id;

  const valSpan = document.createElement("div");
  valSpan.className = "slider-value";
  valSpan.id = "val-" + cfg.id;
  valSpan.textContent = cfg.init + cfg.unit;

  slider.addEventListener("input", () => {
    vals[cfg.id] = parseFloat(slider.value);
    valSpan.textContent = slider.value + cfg.unit;
    clearTimeout(debounce);
    debounce = setTimeout(sendIK, 80);
  });

  row.appendChild(label);
  row.appendChild(bounds);
  row.appendChild(slider);
  row.appendChild(valSpan);

  document.getElementById(cfg.group + "-sliders").appendChild(row);
});

function sendIK() {
  fetch("/ik", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(vals)
  })
  .then(r => r.json())
  .then(updateDisplay)
  .catch(err => {
    document.getElementById("status").className = "error";
    document.getElementById("status").textContent = "Network error";
  });
}

function updateDisplay(data) {
  const statusEl = document.getElementById("status");

  if (data.status === "error") {
    statusEl.className = "error";
    statusEl.textContent = "IK error: " + data.message;
    return;
  }

  if (data.status === "in_reach") {
    statusEl.className = "in-reach";
    statusEl.textContent = "IN REACH — servos updated";
  } else {
    statusEl.className = "out-reach";
    statusEl.textContent = "OUT OF REACH: " + data.out_of_range.join(", ");
  }

  const tbody = document.getElementById("servo-table");
  tbody.innerHTML = "";

  for (const [name, s] of Object.entries(data.servos)) {
    const tr = document.createElement("tr");
    const statusClass = s.ok ? "ok" : "bad";
    const statusText  = s.ok ? "✓" : "✗ out of range";
    tr.innerHTML = `
      <td>${name}</td>
      <td>${s.kin > 0 ? "+" : ""}${s.kin.toFixed(1)}°</td>
      <td>${s.cmd}°</td>
      <td class="dim">${s.min} – ${s.max}</td>
      <td class="${statusClass}">${statusText}</td>
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
    data = request.json
    target_pos   = {"x": float(data["x"]), "y": float(data["y"]), "z": float(data["z"])}
    target_pitch = math.radians(float(data["pitch"]))
    target_yaw   = math.radians(float(data["yaw"]))

    try:
        ik = inverse_kinematics(target_pos, target_pitch, target_yaw)
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

    out_of_range = []
    servos = {}

    for name in SERVO_NAMES:
        if name not in ik:
            continue
        sdef = SERVO_DEFS[name]
        kin  = ik[name]
        cmd  = kin_to_cmd(sdef, kin)
        ok   = sdef.limits[0] <= cmd <= sdef.limits[1]
        servos[name] = {
            "kin": round(kin, 1),
            "cmd": round(cmd),
            "min": sdef.limits[0],
            "max": sdef.limits[1],
            "ok":  ok,
        }
        if not ok:
            out_of_range.append(name)

    if not out_of_range:
        for name, s in servos.items():
            send_servo(SERVO_IDS[name], s["cmd"])

    response = {
        "status":       "in_reach" if not out_of_range else "out_of_reach",
        "out_of_range": out_of_range,
        "servos":       servos,
        "target":       data,
    }
    app.config["last_state"] = response
    return jsonify(response)


@app.route("/state", methods=["GET"])
def state():
    """Returns the last computed servo state — readable by any network client."""
    return jsonify(app.config.get("last_state", {}))


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
