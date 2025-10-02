#!/bin/bash

# Path to your Flutter app
APP_PATH="/home/jatin/AutoNav/LinuxApp/autonav_linux/build/linux/arm64/release/bundle/autonav_linux"

# Start the app once in background
"$APP_PATH" &
APP_PID=$!

# Wait 10 seconds
sleep 60

# Kill the first instance
kill $APP_PID

# Wait a couple of seconds for cleanup
sleep 2

# Start the app again in foreground so systemd tracks it
exec "$APP_PATH"
