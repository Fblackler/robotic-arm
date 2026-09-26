from flask import Flask, render_template_string, request
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Servo Controller</title>
    <style>
        body {
            font-family: Arial;
            padding: 20px;
            background: #111;
            color: white;
        }

        .slider-container {
            margin-bottom: 25px;
        }

        input[type=range] {
            width: 400px;
        }
    </style>
</head>
<body>
    <h1>Robot Arm Controller</h1>

    {% for i in range(6) %}
    <div class="slider-container">
        <label>Servo {{i}}</label><br>
        <input type="range"
               min="0"
               max="270"
               value="135"
               oninput="sendServo({{i}}, this.value)">
        <span id="val{{i}}">135</span>
    </div>
    {% endfor %}

<script>
function sendServo(index, value) {
    document.getElementById("val"+index).innerText = value;

    fetch("/servo", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({
            servo: index,
            angle: value
        })
    });
}
</script>

</body>
</html>
"""

@app.route("/")
def home():
    return render_template_string(HTML)

@app.route("/servo", methods=["POST"])
def servo():
    data = request.json

    servo = int(data["servo"])
    angle = int(data["angle"])

    cmd = f"{servo}:{angle}\n"
    ser.write(cmd.encode())

    return "OK"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)