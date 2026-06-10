# Metal Arm MoveIt 2

## Dependencies

- Ubuntu 22.04 LTS
- ROS 2 Humble

Install MoveIt 2:

```bash
sudo apt install ros-humble-moveit*
```

Install controller dependencies:

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

## Controlling a Real Arm with MoveIt

Install the Python SDK:

```bash
cd ~/metal_sdk/
pip install .
```

Build the ROS 2 workspace:

```bash
cd ~/metal_ros2/
bash build.sh
```

Start the arm controller:

```bash
cd ~/metal_ros2/
source install/setup.bash
ros2 launch metal_controller single_arm_control.launch.py
```

Start MoveIt 2:

```bash
cd ~/metal_ros2
source install/setup.bash
```

Without gripper:

```bash
ros2 launch metal_no_gripper_moveit demo.launch.py
```

With gripper:

```bash
ros2 launch metal_with_gripper_moveit demo.launch.py
```

In RViz, use the end-effector marker to choose a target pose, then use the MotionPlanning panel to plan and execute.

## Controlling Simulation with MoveIt

Start Gazebo first. See `../metal_gazebo/README.md`.

Then source the workspace:

```bash
cd ~/metal_ros2
source install/setup.bash
```

Use these simulation-specific launches after Gazebo is running. These are not the real-hardware `demo.launch.py` files.

With gripper:

```bash
ros2 launch metal_with_gripper_moveit metal_moveit.launch.py
```

Without gripper:

```bash
ros2 launch metal_no_gripper_moveit metal_moveit.launch.py
```

If Gazebo does not exit cleanly with Ctrl-C:

```bash
pkill -9 -f gzclient
pkill -9 -f gzserver
```
