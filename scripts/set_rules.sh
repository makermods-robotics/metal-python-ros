#!/bin/bash

sudo cp makermods_metal_can.rules /etc/udev/rules.d/

sudo chmod +x /etc/udev/rules.d/makermods_metal_can.rules

sudo udevadm control --reload-rules && sudo udevadm trigger