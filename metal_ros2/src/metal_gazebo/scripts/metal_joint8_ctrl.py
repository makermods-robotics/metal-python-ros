#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperMirrorController(Node):
    def __init__(self):
        super().__init__('gripper_mirror_controller')

        # Subscribe to joint7 state.
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/gripper_controller/controller_state',
            self.joint_state_callback,
            10
        )

        # Publish joint8 commands.
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint8_controller/joint_trajectory',
            10
        )

        # Timer controlling publish frequency.
        self.timer = self.create_timer(0.02, self.publish_joint8_command)

        self.joint7_position = None  # Stores joint7 position.

    def joint_state_callback(self, msg):
        try:
            # Find joint7 index.
            joint_index = msg.joint_names.index("joint7")
            self.joint7_position = msg.reference.positions[joint_index]

        except ValueError:
            self.get_logger().warn("joint7 not found in /gripper_controller/state")

    def publish_joint8_command(self):
        if self.joint7_position is not None:
            # Mirror the joint value.
            joint8_position = -self.joint7_position

            # Create JointTrajectory message.
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ["joint8"]

            # Set the trajectory point.
            point = JointTrajectoryPoint()
            point.positions = [joint8_position]

            traj_msg.points.append(point)

            # Publish to joint8_controller.
            self.publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperMirrorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
