# Metal Python ROS 1

Python SDK and ROS 1 Noetic driver stack for the MakerMods Metal arm.

This branch targets Ubuntu 20.04 and ROS Noetic. The upstream package and binary identifiers still use `metal_*` names for compatibility with the bundled native SDK, ROS package graph, and MoveIt configuration.

The original Chinese README from the imported upstream branch is preserved as `README.zh-CN.md`.

## Platform Support

- Ubuntu 20.04 LTS
- ROS Noetic
- Linux x86_64 native SDK binary

The Python package setup script contains an arm64 selection path, but this branch only includes `metal_sdk/metal_sdk/lib/x64/libmetal_sdk_x64.so`. arm64/aarch64 support requires a compatible `libmetal_sdk_arm64.so` from MakerMods or a rebuilt SDK.

## Dependencies

```bash
sudo apt update
sudo apt install can-utils net-tools iproute2 libgoogle-glog-dev
sudo apt install ros-noetic-kdl-parser liborocos-kdl-dev ros-noetic-urdf ros-noetic-trac-ik
```

## CAN Setup

Configure the USB-CAN device once:

```bash
cd metal_ros/can_scripts/
bash set_only_one_can.sh
```

Start CAN after rebooting the computer or reconnecting the USB-CAN adapter:

```bash
bash can_scripts/start_can0.sh
```

Multi-arm setups use `can0` through `can3`; see the scripts in `metal_ros/can_scripts/`.

## Install the Python SDK

```bash
cd metal_sdk/
pip install .
```

The Python package exposes `MetalSDKInterface` and `ControlMode`. The hardware-control implementation is inside the bundled native shared library.

## Build the ROS 1 Workspace

```bash
cd metal_ros/
bash build.sh
source devel/setup.bash
```

## Launch Examples

Single arm in gravity-compensation mode for teaching or data capture:

```bash
roslaunch metal_controller one_master.launch
```

Single arm in normal control mode for joint-position or end-pose commands:

```bash
roslaunch metal_controller single_arm_control.launch
```

Two arms in normal control mode:

```bash
roslaunch metal_controller two_arm_control.launch
```

One leader arm and one follower arm:

```bash
roslaunch metal_controller one_master_slave.launch
```

Two leader/follower pairs:

```bash
roslaunch metal_controller two_master_slave.launch
```

## Topic Control Examples

Send a joint-position command:

```bash
source devel/setup.bash
bash control_scripts/joint_position_control.sh
```

Move to zero position:

```bash
source devel/setup.bash
bash control_scripts/go_zero_position.sh
```

Send an end-pose command:

```bash
source devel/setup.bash
bash control_scripts/end_pose_control.sh
```

## Compatibility Note

This repository is a clean local import, not a GitHub fork. It keeps `metal_*` package names and `MetalSDKInterface` symbols because those names are coupled to the current SDK binary and ROS package graph. User-facing documentation and descriptions should use MakerMods and Metal arm naming.
