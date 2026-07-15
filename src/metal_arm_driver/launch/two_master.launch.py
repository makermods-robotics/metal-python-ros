import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

params_file = os.path.join(
    get_package_share_directory('metal_arm_driver'), 'config', 'two_master.yaml')

right_arm_node = Node(
    package='metal_arm_driver',
    executable='metal_arm_driver',
    name='right_master_arm',
    output='screen',
    parameters=[params_file]
)

left_arm_node = Node(
    package='metal_arm_driver',
    executable='metal_arm_driver',
    name='left_master_arm',
    output='screen',
    parameters=[params_file]
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        right_arm_node,
        left_arm_node
    ])