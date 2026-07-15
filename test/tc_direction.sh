#!/usr/bin/env bash
# [READ-ONLY] Per-joint direction check for gravity-comp calibration (motors OFF).
# Guides you to move each gravity-loaded joint the way gravity pulls it and flags
# joints whose motor direction is flipped vs the URDF.
source "$(dirname "$0")/_env.sh"
/usr/bin/python3 "$(dirname "$0")/calib_direction.py" "$@"
