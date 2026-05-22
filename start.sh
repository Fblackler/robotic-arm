#!/bin/bash
PORT=7682

pkill -f "ttyd.*-p $PORT" 2>/dev/null
sleep 1

IP=$(hostname -I | awk '{print $1}')

# Detect if WSL mirrored networking is not active (172.x internal IP)
if [[ "$IP" == 172.* ]]; then
    echo ""
    echo "  ERROR: WSL is using NAT networking — other devices can't reach this terminal."
    echo ""
    echo "  Fix (run in PowerShell on this machine, no admin needed):"
    echo '  echo "[wsl2]`nnetworkingMode=mirrored" > "$env:USERPROFILE\.wslconfig"'
    echo "  wsl --shutdown"
    echo ""
    echo "  Then reopen WSL and run this script again."
    echo ""
    exit 1
fi

echo ""
echo "  WSL Web Terminal"
echo "  ================"
echo ""
echo "  Open on any device on your network:"
echo "  http://$IP:$PORT"
echo ""
echo "  Press Ctrl+C to stop."
echo ""

ttyd -W -p $PORT -t fontSize=16 bash -l
