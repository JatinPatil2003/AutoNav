#!/bin/bash
echo "$(date) - Joystick disconnected" >> /home/jatin/AutoNav/autonav_utils/joystick.log
docker stop joystick_controller
