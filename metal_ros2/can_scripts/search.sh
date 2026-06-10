#!/bin/bash

if [ -e /dev/ttyACM* ]; then
  echo "Found ttyACM device"
else
  echo "No ttyACM device found"
  exit 1
fi

udevadm info -a -n /dev/ttyACM* | grep idVendor | awk 'NR==1 {print substr($0, 10,23)}'

udevadm info -a -n /dev/ttyACM* | grep idProduct | awk 'NR==1 {print substr($0, 10,23)}'

udevadm info -a -n /dev/ttyACM* | grep serial | awk 'NR==1 {print substr($0, 10)}'