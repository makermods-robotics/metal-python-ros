#!/usr/bin/env bash
# READ-ONLY: read joint state directly via metal_sdk. Arm never energized.
source "$(dirname "$0")/_env.sh"
/usr/bin/python3 "$(dirname "$0")/metal_test.py" read "$@"
