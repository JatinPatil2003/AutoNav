#!/bin/bash

# Check if user is root/running with sudo
if [ "$(whoami)" != root ]; then
  echo Please run this script with sudo
  exit
fi

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

cp "${CURR_DIR}"/99-autonav.rules /etc/udev/rules.d/99-autonav.rules
echo "usb rules file install at /etc/udev/rules.d/99-autonav.rules"

echo "reload udev rules"
udevadm control --reload-rules && udevadm trigger
echo "udev rules reload done"
echo "exit"



# udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSB0)