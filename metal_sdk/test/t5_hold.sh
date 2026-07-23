#!/usr/bin/env bash
# ENERGIZES: enable + command CURRENT position. Arm should stay put.
# Validates the command path with minimal motion.
source "$(dirname "$0")/_env.sh"
/usr/bin/python3 "$(dirname "$0")/metal_test.py" hold "$@"
