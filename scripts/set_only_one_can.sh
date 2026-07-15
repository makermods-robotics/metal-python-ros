#!/bin/bash
workspace=$(pwd)

if [ -e /dev/ttyACM* ]; then
  echo "Found ttyACM device"
else
  echo "No ttyACM device found"
  exit 1
fi

idVendor=$(udevadm info -a -n /dev/ttyACM* | grep idVendor | awk 'NR==1 {print substr($0, 23,4)}')
idProduct=$(udevadm info -a -n /dev/ttyACM* | grep idProduct | awk 'NR==1 {print substr($0, 24,4)}')
serial_number=$(udevadm info -a -n /dev/ttyACM* | grep serial | awk -F'"' 'NR==1 {print $2}')

echo -e "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", ATTRS{serial}==\"$serial_number\", SYMLINK+=\"makermods_metal_can0\"" > makermods_metal_can.rules

sudo cp makermods_metal_can.rules /etc/udev/rules.d/

sudo chmod +x /etc/udev/rules.d/makermods_metal_can.rules

sudo udevadm control --reload-rules && sudo udevadm trigger

echo "/dev/makermods_metal_can0 symlink created!"