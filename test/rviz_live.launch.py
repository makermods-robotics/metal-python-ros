"""Live RViz: real arm -> RViz model tracking.

Starts: driver node (READ-ONLY, auto_enable=false -> motors OFF, arm limp &
hand-movable) + arm_state_bridge + robot_state_publisher + RViz. Move the real
arm by hand; the RViz model follows.

Run via test/t7_rviz_live.sh (sets up env).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory("metal_arm_description")
    urdf = os.path.join(desc, "urdf", "metal_no_gripper.urdf")
    rviz = os.path.join(desc, "rviz", "metal_no_gripper.rviz")
    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)
    here = os.path.dirname(os.path.abspath(__file__))

    return LaunchDescription([
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="metal_arm_driver", executable="metal_arm_driver",
            name="metal_arm_driver_node", output="screen",
            parameters=[{
                "arm_can_id": "can0",
                "arm_control_type": "leader_arm",  # publishes state; no control sub
                "arm_end_type": 0,
                "auto_enable": False,              # motors OFF -> arm hand-movable
            }],
        ),
        ExecuteProcess(
            cmd=["/usr/bin/python3", os.path.join(here, "arm_state_bridge.py")],
            output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", rviz], output="screen",
        ),
    ])
