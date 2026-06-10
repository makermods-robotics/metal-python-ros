# Metal Arm Teach Mode Scripts

This directory contains ROS 2 `rclpy` scripts for teaching, recording, and replaying Metal arm poses and trajectories. Before running them, source the built workspace:

```bash
source /path/to/metal_ros2/install/setup.bash
```

## `normal_mode/`

Use these scripts for trajectory recording and playback in normal control mode, where the lower-level controller performs interpolation and smoothing.

| Script | Purpose |
|---|---|
| `record_trajectory.py` | Subscribe to joint feedback and write `position`, `velocity`, `effort`, and `end_pose` samples to JSONL. |
| `joint_position_play_trajectory.py` | Replay joint positions from JSONL. A first-row `position` length of 6 means arm-only; length 7 means arm plus gripper stroke. |
| `end_pose_play_trajectory.py` | Replay end-effector poses from JSONL, using the same 6/7-value gripper convention. |

Record a trajectory:

```bash
source metal_ros2/install/setup.bash
ros2 launch metal_controller one_master.launch.py
cd /path/to/metal_teach_mode/normal_mode
python3 record_trajectory.py
```

Replay through normal control mode:

```bash
source metal_ros2/install/setup.bash
ros2 launch metal_controller single_arm_control.launch.py
cd /path/to/metal_teach_mode/normal_mode
python3 joint_position_play_trajectory.py
```

or:

```bash
python3 end_pose_play_trajectory.py
```

Common parameters include `joint_state_topic`, `output_jsonl`, `trajectory_jsonl`, `joint_position_control_topic`, `end_pose_control_topic`, `playback_rate_hz`, and `record_rate_hz`.

## `teach_trajectory/`

These scripts are similar to `normal_mode/`, but are intended for higher-rate teaching data such as 400 Hz streams. Joint playback publishes `ArmJointState` to a follower-arm topic.

Record:

```bash
source metal_ros2/install/setup.bash
ros2 launch metal_controller one_master.launch.py
cd /path/to/metal_teach_mode/teach_trajectory
python3 record_trajectory.py
```

Replay to a follower arm:

```bash
source metal_ros2/install/setup.bash
ros2 launch metal_controller one_slave.launch.py
cd /path/to/metal_teach_mode/teach_trajectory
python3 joint_position_play_trajectory.py
```

End-pose replay is also available:

```bash
python3 end_pose_play_trajectory.py
```

Make sure the script parameters match the active `metal_controller` YAML and launch configuration.

## `teach_pose/`

Use these scripts for discrete key-pose teaching instead of continuous trajectories.

| Script | Purpose |
|---|---|
| `record_pos.py` | Interactive recording. Press Enter to save a pose; press `q` to quit. |
| `joint_position_play_pos.py` | Replay saved poses as joint-position commands and wait for each target to be reached. |
| `end_pose_play_pos.py` | Replay saved poses as end-pose commands and wait for each target to be reached. |

Record poses:

```bash
source metal_ros2/install/setup.bash
ros2 launch metal_controller one_master.launch.py
cd /path/to/metal_teach_mode/teach_pose
python3 record_pos.py
```

Replay poses:

```bash
source metal_ros2/install/setup.bash
ros2 launch metal_controller single_arm_control.launch.py
cd /path/to/metal_teach_mode/teach_pose
python3 joint_position_play_pos.py
```

or:

```bash
python3 end_pose_play_pos.py
```

Common parameters include `joint_state_topic`, `joint_position_control_topic`, `end_pose_control_topic`, `poses_jsonl`, `position_tolerance`, `pose_tolerance`, and `sleep_after_reach_sec`.

## Notes

- The scripts require `rclpy`, `metal_msg`, and a running controller node with matching topics.
- Several scripts use `input()`, so run them in a terminal with standard input.
- JSONL paths are often relative paths such as `data/recorded_pos.jsonl`; use absolute paths or launch parameters when running from another directory.
