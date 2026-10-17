#!/usr/bin/env bash
# Waits for RobotApp's nginx container to actually be serving, then launches
# Chrome fullscreen (F11-equivalent) against it. Docker's own restart policy
# brings the container back after a crash/reboot; this script just needs to
# not race ahead of that on boot (robotapp can take a few seconds after
# `docker compose up` before nginx + RobotServer behind it are both
# answering).
set -euo pipefail

# URL="http://localhost/"
URL="http://localhost:5174/"

until curl --silent --fail --max-time 2 "$URL" >/dev/null 2>&1; do
  sleep 1
done

google-chrome \
  --app="$URL" \
  --noerrdialogs \
  --disable-infobars \
  --no-first-run \
  --overscroll-history-navigation=0 \
  --disable-pinch &
CHROME_PID=$!

# --start-fullscreen is unreliable together with --app=, so instead wait for
# the app window to actually appear and send it a real F11 keypress.
for _ in $(seq 1 50); do
  WIN_ID=$(xdotool search --onlyvisible --class "Google-chrome" | head -n1 || true)
  [[ -n "$WIN_ID" ]] && break
  sleep 0.2
done

if [[ -n "${WIN_ID:-}" ]]; then
  xdotool windowactivate "$WIN_ID"
  xdotool key --window "$WIN_ID" F11
fi

wait "$CHROME_PID"
