#!/usr/bin/env bash
# [sudo] Bring up can0 from the CANable (slcan, 1Mbps). Safe (no arm motion).
set -e
DEV=/dev/makermods_metal_can0
[ -e "$DEV" ] || DEV=/dev/ttyACM0
echo "using $DEV"
sudo slcand -o -f -s8 "$DEV" can0
sudo ifconfig can0 up
sudo ip link set can0 txqueuelen 1000
ip -brief link show can0
echo "--- 2s candump (DM motors are request-response; may be empty until SDK talks) ---"
timeout 2 candump can0 | head || true
echo "can0 is up. Next: bash test/t1_read.sh"
