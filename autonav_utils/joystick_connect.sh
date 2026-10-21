#!/bin/bash
echo "$(date) - Joystick connected" >> /home/jatin/AutoNav/autonav_utils/joystick.log
docker start autonav-joystick_controller
