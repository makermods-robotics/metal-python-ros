#!/usr/bin/env bash
# 一键构建 metal_sdk：native(.so) -> 拷贝 -> pybind11 binding
set -eo pipefail
# conda-shadow trap: a conda python ahead of /usr/bin lacks catkin_pkg and breaks
# ament's cmake; force the system python 3.10 (what ROS and the nodes run under).
export PATH=/usr/bin:$PATH
# ROS humble's setup.bash references unset variables internally, which trips
# `set -u`; relax nounset just for the source, then restore it.
set +u
source /opt/ros/humble/setup.bash
set -u
# SDK root = this script's directory (metal_sdk/ at the repo root)
SDK="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
ARCH=$(uname -m)
case "$ARCH" in
  x86_64|AMD64)   ARCHDIR=x64;   LIBNAME=libmetal_sdk_x64.so;;
  aarch64|arm64)  ARCHDIR=arm64; LIBNAME=libmetal_sdk_arm64.so;;
  *) echo "Unsupported arch: $ARCH" && exit 1;;
esac
# 1. 编 native
cd "$SDK" && colcon build --packages-select metal_sdk --base-paths native
# 2. 拷 .so 到 python 包 lib/<arch>/
SO=$(find "$SDK/install/metal_sdk" -name "$LIBNAME" | head -1)
cp "$SO" "$SDK/metal_sdk/lib/$ARCHDIR/"
# 3. 装 pybind11 绑定（固定用 /usr/bin/python3 = 系统 3.10，与 ROS 节点运行时一致；
#    --no-build-isolation 以复用系统已装的 python3-pybind11，而非联网拉取 PyPI 包）
/usr/bin/python3 -m pip install --user --no-build-isolation "$SDK/binding"
echo "metal_sdk built for $ARCHDIR"
