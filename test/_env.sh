#!/usr/bin/env bash
# shared env for metal hardware tests — `source` this.
# Fixes the conda-shadow trap: prepend /usr/bin so python3 -> 3.10 (ROS's python).
conda deactivate 2>/dev/null || true
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
