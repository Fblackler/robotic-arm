# WSL Web Terminal Launcher
# Exposes your WSL terminal as a browser-accessible web terminal on the local network.
# Must run as Administrator (netsh requires it). Will auto-elevate if needed.

if (-not ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Start-Process powershell -Verb RunAs -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit
}

# ── Config ────────────────────────────────────────────────────────────────────
$PORT   = 7682
$DISTRO = ""       # Leave empty to use the default WSL distro (currently Ubuntu)
                   # Set to e.g. "Ubuntu-22.04" or "WSL-ROS2" to pick a specific one
# $AUTH  = "user:pass"   # Uncomment to require login (recommended on shared networks)
# ─────────────────────────────────────────────────────────────────────────────

$wslArgs = if ($DISTRO) { @("-d", $DISTRO) } else { @() }

Write-Host ""
Write-Host "  WSL Web Terminal" -ForegroundColor Cyan
Write-Host "  ================" -ForegroundColor Cyan
Write-Host ""

# ── 1. Install ttyd inside WSL if missing ─────────────────────────────────────
Write-Host "  [1/4] Checking ttyd in WSL..." -ForegroundColor White
$ttydPath = & wsl @wslArgs bash -c "which ttyd 2>/dev/null"
if (-not $ttydPath) {
    Write-Host "        ttyd not found - installing via apt..." -ForegroundColor Yellow
    & wsl @wslArgs bash -c "sudo apt-get update -qq && sudo apt-get install -y ttyd"
    $ttydPath = & wsl @wslArgs bash -c "which ttyd 2>/dev/null"
    if (-not $ttydPath) {
        Write-Host "        ERROR: ttyd install failed. Try manually: sudo apt install ttyd" -ForegroundColor Red
        Read-Host "`n  Press Enter to exit"
        exit 1
    }
}
Write-Host "        ttyd ready at $ttydPath" -ForegroundColor Green

# Kill any existing ttyd on this port to avoid EADDRINUSE
& wsl @wslArgs bash -c "pkill -f 'ttyd.*-p $PORT' 2>/dev/null; sleep 1" | Out-Null

# ── 2. Get WSL2 IP (changes every boot) ──────────────────────────────────────
Write-Host "  [2/4] Resolving WSL IP..." -ForegroundColor White
$wslIp = (& wsl @wslArgs bash -c "hostname -I 2>/dev/null").Trim().Split(' ')[0]
if (-not $wslIp) {
    Write-Host "        ERROR: Could not get WSL IP. Make sure the distro is not paused." -ForegroundColor Red
    Read-Host "`n  Press Enter to exit"
    exit 1
}
Write-Host "        WSL IP: $wslIp" -ForegroundColor Green

# ── 3. Windows port proxy: host NIC -> WSL ────────────────────────────────────
Write-Host "  [3/4] Configuring port forwarding..." -ForegroundColor White
netsh interface portproxy delete v4tov4 listenport=$PORT listenaddress=0.0.0.0 | Out-Null
netsh interface portproxy add    v4tov4 listenport=$PORT listenaddress=0.0.0.0 connectport=$PORT connectaddress=$wslIp | Out-Null
Write-Host "        0.0.0.0:$PORT  ->  $wslIp`:$PORT" -ForegroundColor Green

# ── 4. Firewall ───────────────────────────────────────────────────────────────
Write-Host "  [4/4] Opening firewall port $PORT..." -ForegroundColor White
netsh advfirewall firewall delete rule name="WSL Web Terminal" protocol=TCP dir=in localport=$PORT 2>$null | Out-Null
netsh advfirewall firewall add    rule name="WSL Web Terminal" protocol=TCP dir=in localport=$PORT action=allow | Out-Null
Write-Host "        Firewall rule set." -ForegroundColor Green

# ── Display access URL ────────────────────────────────────────────────────────
$hostIp = (Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -notmatch "^127\." -and $_.IPAddress -notmatch "^169\.254\." -and $_.InterfaceAlias -notmatch "WSL|Loopback" } |
    Sort-Object -Property { if ($_.InterfaceAlias -match "Wi.?Fi|Wireless|WLAN") { 0 } else { 1 } } |
    Select-Object -First 1).IPAddress

Write-Host ""
Write-Host "  +--------------------------------------------+" -ForegroundColor Yellow
Write-Host "  |  Open in any browser on your network:     |" -ForegroundColor Yellow
Write-Host "  |                                            |" -ForegroundColor Yellow
Write-Host "  |   http://$($hostIp)`:$PORT             |" -ForegroundColor Cyan
Write-Host "  |                                            |" -ForegroundColor Yellow
Write-Host "  +--------------------------------------------+" -ForegroundColor Yellow
Write-Host ""
Write-Host "  ! No authentication is set - anyone on your network" -ForegroundColor Red
Write-Host "  ! can open a full shell. To add a password, uncomment" -ForegroundColor Red
Write-Host "  ! the `$AUTH line at the top of this script." -ForegroundColor Red
Write-Host ""
Write-Host "  Press Ctrl+C in this window to stop the terminal server." -ForegroundColor DarkGray
Write-Host ""

# ── Start ttyd (blocking - this window IS the server process) ─────────────────
$ttydCmd = "ttyd -W -p $PORT -t fontSize=16 bash -l"
if ($AUTH) { $ttydCmd = "ttyd -W -p $PORT -c '$AUTH' -t fontSize=16 bash -l" }

& wsl @wslArgs bash -c $ttydCmd

# ── Cleanup on exit ───────────────────────────────────────────────────────────
Write-Host "`n  Cleaning up port proxy..." -ForegroundColor DarkGray
netsh interface portproxy delete v4tov4 listenport=$PORT listenaddress=0.0.0.0 | Out-Null
Write-Host "  Done." -ForegroundColor DarkGray
Read-Host "`n  Press Enter to close"
