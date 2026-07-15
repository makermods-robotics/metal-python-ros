#!/usr/bin/env bash
# 一键构建 metal_sdk：native(.so) -> 拷贝 -> pybind11 binding
set -eo pipefail
# ROS humble's setup.bash references unset variables internally, which trips
# `set -u`; relax nounset just for the source, then restore it.
set +u
source /opt/ros/humble/setup.bash
set -u
WS="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
ARCH=$(uname -m)
case "$ARCH" in
  x86_64|AMD64)   ARCHDIR=x64;   LIBNAME=libmetal_sdk_x64.so;;
  aarch64|arm64)  ARCHDIR=arm64; LIBNAME=libmetal_sdk_arm64.so;;
  *) echo "Unsupported arch: $ARCH" && exit 1;;
esac
# 1. 编 native
cd "$WS" && colcon build --packages-select metal_sdk
# 2. 拷 .so 到 python 包 lib/<arch>/
SO=$(find "$WS/install/metal_sdk" -name "$LIBNAME" | head -1)
cp "$SO" "$WS/src/metal_sdk/metal_sdk/lib/$ARCHDIR/"
# 3. 装 pybind11 绑定（固定用 /usr/bin/python3 = 系统 3.10，与 ROS 节点运行时一致；
#    --no-build-isolation 以复用系统已装的 python3-pybind11，而非联网拉取 PyPI 包）
/usr/bin/python3 -m pip install --user --no-build-isolation "$WS/src/metal_sdk/binding"
echo "metal_sdk built for $ARCHDIR"
