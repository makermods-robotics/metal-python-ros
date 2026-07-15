#!/usr/bin/env bash
# MOVES: small single-joint move then back (default J1 +0.15rad, slowest speed).
# Override: bash test/t6_move.sh --joint 2 --delta 0.1
source "$(dirname "$0")/_env.sh"
/usr/bin/python3 "$(dirname "$0")/metal_test.py" move "$@"
