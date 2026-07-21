import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

params_file = os.path.join(
    get_package_share_directory('metal_controller'), 'config', 'star_master_slave.yaml')

star_arm_leader_node = Node(
    package='metal_controller',
    executable='star_arm_leader',
    name='star_arm_leader',
    output='screen',
    parameters=[params_file]
)

right_slave_arm_node = Node(
    package='metal_controller',
    executable='metal_controller',
    name='right_slave_arm',
    output='screen',
    parameters=[params_file]
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        star_arm_leader_node,
        right_slave_arm_node
    ])
