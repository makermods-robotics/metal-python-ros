# Metal Python ROS 2

Python SDK and ROS 2 Humble driver stack for the MakerMods Metal arm.

This branch targets Ubuntu 22.04 and ROS 2 Humble. The upstream package and binary identifiers still use `metal_*` names for compatibility with the bundled native SDK, but user-facing documentation refers to the robot as the Metal arm.

The original Chinese README from the imported upstream branch is preserved as `README.zh-CN.md`.

## Quick Start with Drift

You can easily start working with the Metal arm using [Drift](https://godrift.ai) — an
agentic simulation platform for robotics.

Paste this in your terminal and run to install Drift:

```bash
curl -fsSL https://godrift.ai/install | bash
```

From the `/robots` command in the Drift CLI, you can easily preview the Metal arm and
add it to your workspace.

## Platform Support

- Ubuntu 22.04 LTS
- ROS 2 Humble

The native SDK is built from source (`metal_sdk/native/`, a colcon package), so both
x86_64 and arm64/aarch64 are supported — no prebuilt binary is required.

## Dependencies

```bash
sudo apt update
sudo apt install can-utils net-tools iproute2 libgoogle-glog-dev libnlopt-cxx-dev
sudo apt install ros-humble-kdl-parser liborocos-kdl-dev ros-humble-joint-state-publisher-gui
```

## CAN Setup

Configure the USB-CAN device once:

```bash
cd metal_ros2/can_scripts/
bash set_only_one_can.sh
```

Start CAN after rebooting the computer or reconnecting the USB-CAN adapter:

```bash
bash can_scripts/start_can0.sh
```

Multi-arm setups use `can0` through `can3`; see the scripts in `metal_ros2/can_scripts/`.

## Build and Install the Python SDK

One command builds the native SDK from source and installs the Python binding:

```bash
bash metal_sdk/build_metal_sdk.sh
```

This runs colcon on `metal_sdk/native/` (producing `libmetal_sdk_<arch>.so`), copies
the library into the Python package, and pip-installs the pybind11 binding. The
package exposes `MetalSDKInterface` and `ControlMode`. See `metal_sdk/build_sdk.md`
for details and manual steps.

## Build the ROS 2 Workspace

```bash
cd metal_ros2/
bash build.sh
source install/setup.bash
```

## Launch Examples

Single arm in gravity-compensation mode for teaching or data capture:

```bash
ros2 launch metal_controller one_master.launch.py
```

Single arm in normal control mode for joint-position or end-pose commands:

```bash
ros2 launch metal_controller single_arm_control.launch.py
```

Two arms in normal control mode:

```bash
ros2 launch metal_controller two_arm_control.launch.py
```

One leader arm and one follower arm:

```bash
ros2 launch metal_controller one_master_slave.launch.py
```

Two leader/follower pairs:

```bash
ros2 launch metal_controller two_master_slave.launch.py
```

## Star Arm 102 as Leader (Alternative Teleop)

Instead of a second Metal arm, the follower can be teleoperated with a Seeed Studio
Star Arm 102 / reBot Arm 102 leader — a low-cost 7-joint leader arm on FashionStar UART
smart servos, connected over USB serial (default `/dev/ttyUSB0`). The Metal
leader/follower launches above are unchanged; this is an additional option.

Install the servo driver into the Python used by ROS, and make sure your user can
open the serial port (member of the `dialout` group; re-login after adding):

```bash
python3 -m pip install 'motorbridge-smart-servo>=0.0.4,<0.1.0'
sudo usermod -aG dialout $USER   # only needed once
```

Launch the Star Arm leader with a Metal follower on `can0` (bring the CAN interface
up first, e.g. `metal_ros2/can_scripts/start_can0.sh`):

```bash
ros2 launch metal_controller star_master_slave.launch.py
```

The `star_arm_leader` node reads the leader servos, maps them onto the Metal joint
convention (per-joint direction/scale and ranges, configured in
`metal_controller/config/star_master_slave.yaml`), and publishes `ArmJointState`
commands (`[J1..J6 rad, gripper stroke mm]`) on `/master_arm_right/joint_states`
for the standard follower node. On startup it slowly slews the follower from its
current pose to the leader pose before tracking at full speed.

Servo zero points are stored inside the servos themselves. To (re)zero the leader,
hold it in its zero pose and set the origin point of each servo (for example with the
LeRobot `rebot_102_leader` calibration flow, or the FashionStar tooling); this node
does not perform calibration.

## Topic Control Examples

Send a joint-position command:

```bash
source install/setup.bash
bash control_scripts/joint_position_control.sh
```

Move to zero position:

```bash
source install/setup.bash
bash control_scripts/go_zero_position.sh
```

Send an end-pose command:

```bash
source install/setup.bash
bash control_scripts/end_pose_control.sh
```

## Message Types

- `ArmJointState`: joint position, velocity, effort, and end-pose feedback.
- `ArmStatus`: joint names, motor currents, temperatures, and error codes.
- `ArmJointPositionControl`: six-joint position command plus gripper command.
- `ArmEndPoseControl`: end-pose command plus gripper command.

## Compatibility Note

This repository is a clean local import, not a GitHub fork. It keeps `metal_*` package names and `MetalSDKInterface` symbols because those names are coupled to the current SDK binary and ROS package graph. User-facing documentation and descriptions should use MakerMods and Metal arm naming.
