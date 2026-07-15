#!/usr/bin/env bash
# metal-python-ros: 一键安装系统依赖（需要 sudo）
set -euo pipefail

sudo apt update

# --- 构建必需（metal_sdk native + ROS2 包编译，缺一不可） ---
sudo apt install -y liburdfdom-dev liburdfdom-headers-dev liburdfdom-tools python3-pip python3-pybind11 python3-scipy python3-numpy libnlopt-dev libnlopt-cxx-dev libgoogle-glog-dev libeigen3-dev pybind11-dev can-utils ros-humble-kdl-parser ros-humble-orocos-kdl-vendor ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui

# --- GUI / 运行时（RViz、MoveIt 可视化与规划，非编译强制依赖） ---
sudo apt install -y ethtool ros-humble-xacro ros-humble-rviz2 ros-humble-moveit

echo "metal-python-ros deps installed."
