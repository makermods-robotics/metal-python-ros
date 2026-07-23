#!/usr/bin/env bash
# [MOVES] Slider -> real arm teleop. Drag jsp_gui sliders, the real arm follows
# (RELATIVE/jump-safe: starts at current pose, per-joint capped, slowest speed).
# Starts RSP + jsp_gui + RViz + driver(normal_arm,auto_enable) + delta bridge.
source "$(dirname "$0")/_env.sh"
ros2 launch "$(dirname "$0")/slider_control.launch.py"
