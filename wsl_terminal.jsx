// ============================================================
// WSL Web Terminal
// ============================================================
//
// Simple LAN-accessible browser terminal for WSL.
//
// REQUIREMENTS:
//   npm init -y
//   npm install express ws node-pty
//
// RUN:
//   set TERMINAL_PASSWORD=myPassword
//   node wsl_web_terminal.js
//
// ACCESS:
//   http://<windows-ip>:7681
//
// ============================================================

const express = require("express");
const http = require("http");
const WebSocket = require("ws");
const pty = require("node-pty");

const PORT = 7681;
const PASSWORD = process.env.TERMINAL_PASSWORD || "password";

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

// ============================================================
// HTML
// ============================================================

app.get("/", (req, res) => {
    res.send(`
<!DOCTYPE html>
<html>
<head>
    <title>WSL Terminal</title>

    <style>
        body {
            background: black;
            color: #00ff00;
            font-family: monospace;
            margin: 0;
            padding: 0;
        }

        #login {
            padding: 20px;
        }

        #terminal {
            white-space: pre-wrap;
            padding: 10px;
            height: 100vh;
            overflow-y: auto;
            display: none;
        }

        input {
            background: black;
            color: #00ff00;
            border: 1px solid #00ff00;
            font-family: monospace;
            padding: 5px;
        }
    </style>
</head>

<body>

<div id="login">
    <h2>WSL Terminal Login</h2>

    <input
        id="password"
        type="password"
        placeholder="Password"
    >

    <button onclick="connect()">
        Connect
    </button>
</div>

<div id="terminal"></div>

<script>

let socket;

function connect() {

    const password =
        document.getElementById("password").value;

    socket = new WebSocket(
        "ws://" + location.host
    );

    socket.onopen = () => {

        socket.send(
            JSON.stringify({
                type: "auth",
                password: password
            })
        );
    };

    socket.onmessage = (event) => {

        const terminal =
            document.getElementById("terminal");

        const login =
            document.getElementById("login");

        if (event.data === "__AUTH_FAILED__") {

            alert("Wrong password");
            return;
        }

        login.style.display = "none";
        terminal.style.display = "block";

        terminal.textContent += event.data;

        window.scrollTo(
            0,
            document.body.scrollHeight
        );
    };

    document.addEventListener("keydown", (e) => {

        if (!socket) return;

        socket.send(
            JSON.stringify({
                type: "input",
                data: e.key
            })
        );

        e.preventDefault();
    });
}

</script>

</body>
</html>
    `);
});

// ============================================================
// WebSocket
// ============================================================

wss.on("connection", (ws) => {

    let authenticated = false;
    let shell = null;

    ws.on("message", (msg) => {

        let data;

        try {
            data = JSON.parse(msg);
        }
        catch {
            return;
        }

        // ====================================================
        // Authentication
        // ====================================================

        if (data.type === "auth") {

            if (data.password !== PASSWORD) {

                ws.send("__AUTH_FAILED__");
                ws.close();
                return;
            }

            authenticated = true;

            shell = pty.spawn(
                "wsl.exe",
                [],
                {
                    name: "xterm-color",
                    cols: 120,
                    rows: 30,
                    cwd: process.env.HOME,
                    env: process.env
                }
            );

            shell.onData((output) => {
                ws.send(output);
            });

            return;
        }

        // ====================================================
        // Input
        // ====================================================

        if (
            authenticated &&
            data.type === "input"
        ) {

            if (data.data === "Enter") {
                shell.write("\\r");
            }
            else if (data.data === "Backspace") {
                shell.write("\\x7f");
            }
            else if (data.data.length === 1) {
                shell.write(data.data);
            }
        }
    });

    ws.on("close", () => {

        if (shell) {
            shell.kill();
        }
    });
});

// ============================================================
// Start
// ============================================================

server.listen(PORT, "0.0.0.0", () => {

    console.log(
        "WSL web terminal running on:"
    );

    console.log(
        "http://localhost:" + PORT
    );
});