#!/bin/bash
PORT=7682

pkill -f "ttyd.*-p $PORT" 2>/dev/null
sleep 1

IP=$(hostname -I | awk '{print $1}')

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
