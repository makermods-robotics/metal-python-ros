"""Slider -> real arm teleop (MOVES the arm).

Starts: robot_state_publisher + joint_state_publisher_gui (sliders) + RViz +
driver (normal_arm / NRT / auto_enable=true) + the RELATIVE slider_teleop_bridge.

Drag a slider -> the corresponding real joint moves by the same delta (capped,
slowest speed). Jump-safe: the arm starts at its current pose. Run via
test/t8_slider_control.sh.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory("metal_description")
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
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
            parameters=[{"rate": 10.0}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", rviz], output="screen",
        ),
        Node(
            package="metal_controller", executable="metal_controller",
            name="metal_controller_node", output="screen",
            parameters=[{
                "arm_can_id": "can0",
                "arm_control_type": "normal_arm",  # NRT planned motion
                "arm_end_type": 0,
                "auto_enable": True,               # motors ON — arm WILL move
                "is_sim": False,
            }],
        ),
        ExecuteProcess(
            cmd=["/usr/bin/python3", os.path.join(here, "slider_teleop_bridge.py")],
            output="screen",
        ),
    ])
