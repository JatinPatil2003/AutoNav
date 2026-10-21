#!/bin/bash
echo "$(date) - Joystick disconnected" >> /home/jatin/AutoNav/autonav_utils/joystick.log
docker stop autonav-joystick_controller
