# Metal Arm Gazebo

## Dependencies

- Ubuntu 22.04 LTS
- ROS 2 Humble

Install Gazebo and ROS 2 control dependencies:

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
```

Source the workspace:

```bash
cd ~/metal_ros2
source install/setup.bash
```

Launch simulation with gripper:

```bash
ros2 launch metal_gazebo metal_with_gripper_gazebo.launch.py
```

Launch simulation without gripper:

```bash
ros2 launch metal_gazebo metal_no_gripper_gazebo.launch.py
```

When controlling simulation through MoveIt, start Gazebo first and then start MoveIt with `metal_moveit.launch.py`, not `demo.launch.py`.
