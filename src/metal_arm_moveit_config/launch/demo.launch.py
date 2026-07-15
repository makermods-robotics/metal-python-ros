from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node

joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    parameters=[{"publish_rate": 200}]
)

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("metal", package_name="metal_arm_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
