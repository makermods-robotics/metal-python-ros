#!/usr/bin/env bash
# ENERGIZES: gravity-compensation teach mode. Arm holds its weight & is
# back-drivable by hand. It will NOT move on its own. Ctrl-C to stop.
source "$(dirname "$0")/_env.sh"
/usr/bin/python3 "$(dirname "$0")/metal_test.py" gravity "$@"
