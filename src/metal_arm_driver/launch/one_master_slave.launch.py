import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

params_file = os.path.join(
    get_package_share_directory('metal_arm_driver'), 'config', 'one_master_slave.yaml')

right_master_arm_node = Node(
    package='metal_arm_driver',
    executable='metal_arm_driver',
    name='right_master_arm',
    output='screen',
    parameters=[params_file]
)

right_slave_arm_node = Node(
    package='metal_arm_driver',
    executable='metal_arm_driver',
    name='right_slave_arm',
    output='screen',
    parameters=[params_file]
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        right_master_arm_node,
        right_slave_arm_node
    ])