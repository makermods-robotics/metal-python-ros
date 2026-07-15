# metal_arm_ros

MakerMods Metal arm ROS 2 (Humble) workspace. Native C++/Python driver (ROS1 dependency removed) plus MoveIt support.

## Layout

```
metal_arm_ros/
├── scripts/                    # CAN bring-up scripts, control example scripts, install_deps.sh
├── docs/
│   └── metal_sdk_known_limitations.md   # metal_sdk known limitations (read before use)
└── src/
    ├── metal_sdk/               # native lib + pybind11 binding (not a ROS package, builds separately)
    ├── metal_arm_msgs/          # ROS 2 message definitions
    ├── metal_arm_description/   # URDF / meshes / rviz
    ├── metal_arm_driver/        # driver node (metal_sdk backend)
    └── metal_arm_moveit_config/ # MoveIt configuration
```

## Build procedure (validated order — do not skip or reorder)

### 1. Install system dependencies

```bash
bash scripts/install_deps.sh
```

Requires `sudo`; enter your password when prompted. Two groups: build-critical deps (urdfdom, pybind11, nlopt, glog, eigen, can-utils, ROS2 kdl/robot_state_publisher, etc.) and GUI/runtime deps (ethtool, xacro, rviz2, moveit — used for visualization and planning, not required to compile).

### 2. Build-environment note (important)

If conda / pyenv is active it hijacks which `python3`/`pip` gets used and breaks the ROS build (missing system packages, wrong interpreter). Before building, always run:

```bash
conda deactivate 2>/dev/null; export PATH=/usr/bin:$PATH; source /opt/ros/humble/setup.bash
```

Make sure `python3` resolves to `/usr/bin/python3` (system 3.10, matching ROS Humble), and re-source `/opt/ros/humble/setup.bash` in every new terminal before building or running.

### 3. Build metal_sdk (native lib + Python binding)

```bash
bash src/metal_sdk/build_metal_sdk.sh
```

This script: builds the native `.so` (via `colcon build --packages-select metal_sdk`) → copies the `.so` into `src/metal_sdk/metal_sdk/lib/<arch>/` → installs the pybind11 binding package `metal_sdk` with `/usr/bin/python3 -m pip install --user --no-build-isolation`.

Note: `import metal_sdk` requires ROS to be sourced first (`source /opt/ros/humble/setup.bash`), since the native lib links `rclcpp` — without it the import fails with a missing shared-library error.

### 4. Build the ROS 2 packages

```bash
cd ~/makermods/metal_arm_ros && colcon build
```

This builds the 4 ROS 2 packages: `metal_arm_msgs`, `metal_arm_description`, `metal_arm_driver`, `metal_arm_moveit_config` (`metal_sdk` was already built separately in step 3; colcon will touch it again here but that is idempotent).

### 5. Usage

Source the overlay, then launch one of the driver launch files:

```bash
source ~/makermods/metal_arm_ros/install/setup.bash && ros2 launch metal_arm_driver one_master.launch.py
```

Available launch files (under `src/metal_arm_driver/launch/`):

- `one_master.launch.py` — single master arm
- `one_slave.launch.py` — single slave arm
- `one_master_slave.launch.py` — one master + one slave (teleop pair)
- `two_master.launch.py` — two master arms
- `two_master_slave.launch.py` — two masters + two slaves
- `single_arm_control.launch.py` — single-arm control (MoveIt/control pipeline)
- `two_arm_control.launch.py` — dual-arm control

The CAN interface must be brought up first, e.g.:

```bash
sudo bash scripts/start_can0.sh
```

`scripts/` also provides `search.sh` (probe USB-CAN devices), `set_only_one_can.sh` / `set_rules.sh` (generate/install udev rules for a stable device symlink), and `joint_position_control.sh` / `end_pose_control.sh` / `go_zero_position.sh` (`ros2 topic pub` control examples).

### 6. Known limitations

Read [`docs/metal_sdk_known_limitations.md`](docs/metal_sdk_known_limitations.md) before use — it documents substantive unfinished work carried over from TODOs in the SDK source (collision detection not wired into the control-dispatch path, velocity/jerk limits are placeholder values, joint soft limits are not unconditionally enforced, the trac_ik static library is x86-64-only, etc.).

## 中文文档

见 [README.md](README.md)。

> Prerequisite: this machine already has ROS 2 Humble desktop and colcon installed (`ros-humble-desktop` / `python3-colcon-common-extensions`); `install_deps.sh` only adds this project's extra deps, not ROS itself.

> How the C++ source works (architecture / build / API / how to modify): see `docs/metal_sdk_cpp_source_guide.md`.
