#!/bin/bash
# launch_dashboard.sh
# Starts the mission server and opens the dashboard in Chromium kiosk mode.
# Run this from RetroPie's Ports menu or from the terminal.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVER_SCRIPT="$SCRIPT_DIR/mission_server.py"
DASHBOARD_URL="http://localhost:8080/"

echo "[LAUNCH] Starting mission server..."

# Kill any existing mission server
pkill -f mission_server.py 2>/dev/null
sleep 1

# Start the mission server in the background
python3 "$SERVER_SCRIPT" &
SERVER_PID=$!
echo "[LAUNCH] Mission server PID: $SERVER_PID"

# Give server a moment to start
sleep 2

echo "[LAUNCH] Opening dashboard in Chromium..."

# Launch Chromium in kiosk mode (fullscreen, no UI chrome)
chromium-browser \
  --kiosk \
  --no-sandbox \
  --disable-infobars \
  --disable-session-crashed-bubble \
  --disable-restore-session-state \
  --autoplay-policy=no-user-gesture-required \
  "$DASHBOARD_URL"

# When Chromium closes, also stop the server
echo "[LAUNCH] Chromium closed. Stopping mission server..."
kill $SERVER_PID 2>/dev/null
echo "[LAUNCH] Done."
