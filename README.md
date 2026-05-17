# robotic-arm
Open-source repository featuring detailed 3D schematics for a fully functional robotic arm, including CAD files ready for printing and assembly. The project also provides complete control code, firmware, and documentation to power motors, manage sensors, and execute precise movements.

---

## WSL Web Terminal

Exposes your WSL terminal as a browser-accessible terminal on your local network, so you can control the arm from any device.

### Requirements

- Windows 11 with WSL2 (Ubuntu)
- Admin privileges (for port forwarding and firewall rules)

### How to Run

1. Open PowerShell **as Administrator** (Win + X → Terminal (Admin))
2. Run:

```powershell
powershell -ExecutionPolicy Bypass -File "start.ps1"
```

3. Enter your WSL (sudo) password if prompted on first run to install `ttyd`
4. Open the URL shown in the terminal on any device on your local network:

```
http://192.168.10.202:7681
```

### Stopping

Press **Ctrl+C** in the PowerShell window. The port forwarding rule is cleaned up automatically.

### Notes

- Only accessible on your local network (WiFi)
- No authentication by default — anyone on your network can access a full shell
- To add a password, uncomment the `$AUTH = "user:pass"` line at the top of `start.ps1`
- Tab completion: use **Ctrl+I** as an alternative to Tab if Tab doesn't respond
