#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from metal_arm_msgs.msg import ArmStatus, ArmJointState, ArmEndPoseControl, ArmJointPositionControl
from std_msgs.msg import String
from metal_sdk import MetalSDKInterface, ControlMode
from sensor_msgs.msg import JointState

import os
from ament_index_python.packages import get_package_share_directory

class MetalController(Node):
    def __init__(self):
        super().__init__("metal_arm_driver_node")

        # Declare and read ROS 2 parameters
        self.declare_parameter("arm_can_id", "can0")
        self.declare_parameter("arm_feedback_rate", 200)
        self.declare_parameter("arm_end_pose_control_topic", "/metal/arm_end_pose_control")
        self.declare_parameter("arm_joint_position_control_topic", "/metal/arm_joint_position_control_topic")
        self.declare_parameter("arm_joint_state_topic", "/metal/arm_joint_state")
        self.declare_parameter("arm_status_topic", "/metal/arm_status")
        self.declare_parameter("arm_control_type", "follower_arm")
        self.declare_parameter("arm_end_type", 0)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter('is_sim', False)

        self.can_id = self.get_parameter("arm_can_id").value
        self.arm_feedback_rate = self.get_parameter("arm_feedback_rate").value
        self.arm_end_pose_control_topic = self.get_parameter("arm_end_pose_control_topic").value
        self.arm_joint_position_control_topic = self.get_parameter("arm_joint_position_control_topic").value
        self.arm_joint_state_topic = self.get_parameter("arm_joint_state_topic").value
        self.arm_status_topic = self.get_parameter("arm_status_topic").value
        self.arm_control_type = self.get_parameter("arm_control_type").value
        self.arm_end_type = self.get_parameter("arm_end_type").value
        self.auto_enable = self.get_parameter("auto_enable").value
        self.is_sim = self.get_parameter("is_sim").value

        # URDF path
        package_name = "metal_arm_driver"
        package_share_dir = get_package_share_directory(package_name)

        urdf_files = {
            0: "metal_no_gripper.urdf",
            1: "metal_with_gripper.urdf",
            2: "metal_with_gripper.urdf",
            3: "metal_with_gripper.urdf"
        }

        if self.arm_end_type not in urdf_files:
            self.get_logger().error(f"arm_end_type {self.arm_end_type} not supported")
            raise RuntimeError("Unsupported arm_end_type")

        urdf_filename = urdf_files[self.arm_end_type]
        urdf_path = os.path.join(package_share_dir, "urdf", urdf_filename)

        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF file not found: {urdf_path}")
            raise RuntimeError(f"URDF file not found: {urdf_path}")

        # Initialize the Metal SDK.
        self.metal_interface = MetalSDKInterface(
            can_id=self.can_id,
            urdf_path=urdf_path,
            arm_end_type=self.arm_end_type,
            enable_arm=self.auto_enable,
        )
        if not self.metal_interface.Init():
            self.get_logger().error("Init Metal SDK interface failed")
            raise RuntimeError("Metal SDK Init failed")

        # QoS configuration
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Set control mode
        if self.arm_control_type == "leader_arm":
            self.metal_interface.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
        elif self.arm_control_type == "follower_arm":
            self.metal_interface.SetArmControlMode(ControlMode.RT_JOINT_POSITION)
            self.arm_end_pose_sub = self.create_subscription(
                ArmEndPoseControl,
                self.arm_end_pose_control_topic,
                self.arm_end_pose_callback,
                qos_profile
            )
            self.arm_joint_pos_sub = self.create_subscription(
                ArmJointState,
                self.arm_joint_position_control_topic,
                self.follow_arm_joint_callback,
                qos_profile
            )
        elif self.arm_control_type == "normal_arm":
            self.metal_interface.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)

            # self.arm_end_pose_sub = self.create_subscription(
            #     ArmEndPoseControl,
            #     self.arm_end_pose_control_topic,
            #     self.arm_end_pose_callback,
            #     qos_profile
            # )
            # self.arm_joint_pos_sub = self.create_subscription(
            #     ArmJointPositionControl,
            #     self.arm_joint_position_control_topic,
            #     self.arm_joint_position_callback,
            #     qos_profile
            # )
            print("is_sim:", self.is_sim)
            if self.is_sim:
                self.arm_joint_pos_sub = self.create_subscription(
                JointState,
                "/joint_states",
                self.sim_joint_position_callback,
                qos_profile
                )
            else:
                self.arm_end_pose_sub = self.create_subscription(
                    ArmEndPoseControl,
                    self.arm_end_pose_control_topic,
                    self.arm_end_pose_callback,
                    qos_profile
                )
                self.arm_joint_pos_sub = self.create_subscription(
                    ArmJointPositionControl,
                    self.arm_joint_position_control_topic,
                    self.arm_joint_position_callback,
                    qos_profile
                )
        else:
            self.get_logger().error(f"arm_control_type {self.arm_control_type} not supported")
            raise RuntimeError("Unsupported arm_control_type")

        # Publishers
        self.arm_joint_state_pub = self.create_publisher(
            ArmJointState,
            self.arm_joint_state_topic,
            qos_profile
        )
        self.arm_status_pub = self.create_publisher(
            ArmStatus,
            self.arm_status_topic,
            qos_profile
        )

        # Feedback timer
        timer_period = 1.0 / self.arm_feedback_rate
        self.timer = self.create_timer(timer_period, self.arm_information_timer_callback)

        self.get_logger().info("Metal controller initialized successfully!")

    # ---------------- Callbacks ----------------
    def arm_end_pose_callback(self, msg: ArmEndPoseControl):
        arm_end_pose = list(msg.end_pose[:6])
        self.metal_interface.SetArmEndPose(arm_end_pose)
        self.metal_interface.SetGripperStroke(msg.gripper_stroke, msg.gripper_velocity)

    def follow_arm_joint_callback(self, msg: ArmJointState):
        if len(msg.joint_position) >= 6:
            self.metal_interface.SetFollowerArmJointPosition(msg.joint_position)
        else:
            self.get_logger().error("follow arm receive joint control size < 6")

    def arm_joint_position_callback(self, msg: ArmJointPositionControl):
        # Control J1-J6
        arm_joint_position = list(msg.joint_position[:6])
        self.metal_interface.SetArmJointPosition(arm_joint_position, msg.joint_velocity)
        # Control gripper
        self.metal_interface.SetGripperStroke(msg.gripper_stroke, msg.gripper_velocity)

    def sim_joint_position_callback(self, msg):
        """
        Callback function for joint angles (SimPositionControl)
        Solves joint order mismatch by using a name-to-position map.
        """
        # 1. Create a dictionary to store joint name to position mapping
        # Corresponds to C++: std::map<std::string, double> joint_positions_map;
        joint_positions_map = {}

        # Variable to store gripper joint value
        gripper_pos_raw = 0.0
        gripper_found = False

        # 2. Iterate through msg.name to map positions
        if len(msg.name) != len(msg.position):
            self.get_logger().error("JointState name and position size mismatch!")
            return

        for i, name in enumerate(msg.name):
            pos = msg.position[i]

            # Store in dictionary
            joint_positions_map[name] = pos

            # Strategy: Prefer index 6 as gripper (compatible with old logic)
            if i == 6:
                gripper_pos_raw = msg.position[i]
                gripper_found = True

            # Extra insurance: If joint name contains "gripper" or is "joint7", treat as gripper
            if 'gripper' in name or name == 'joint7':
                gripper_pos_raw = msg.position[i]
                gripper_found = True

        # 3. Dynamically control joints using joint names (Core fix)
        # Build a correctly ordered list ensuring index 0 is joint1, index 1 is joint2, etc.
        arm_joint_position = [0.0] * 6

        # Define expected joint name order
        expected_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        for i in range(6):
            target_name = expected_names[i]

            # Look up in map
            if target_name in joint_positions_map:
                arm_joint_position[i] = joint_positions_map[target_name]
            else:
                # If joint not found, warn and use 0
                self.get_logger().warn_throttle(
                    1.0,
                    f"Joint '{target_name}' not found in JointState message!"
                )
                arm_joint_position[i] = 0.0

        # 4. Send arm control command
        # Now arm_joint_position order is forced to joint1~joint6
        # Assuming self.metal_interface is available in your class
        self.metal_interface.SetArmJointPosition(arm_joint_position, 6)

        # 5. Gripper control
        if gripper_found:
            gripper_stroke = -gripper_pos_raw * 2000.0

            # Optional: NaN check
            import math
            if math.isnan(gripper_stroke):
                gripper_stroke = 0.0
                self.get_logger().warn("Gripper position is NaN, using default.")

            self.metal_interface.SetGripperStroke(gripper_stroke, 6)

    def arm_information_timer_callback(self):
        # Publish joint state
        arm_joint_state = ArmJointState()
        arm_joint_state.header.stamp = self.get_clock().now().to_msg()

        arm_end_pose = self.metal_interface.GetArmEndPose()
        joint_position = self.metal_interface.GetJointPosition()
        joint_velocity = self.metal_interface.GetJointVelocity()
        joint_effort = self.metal_interface.GetJointEffort()

        arm_joint_state.joint_position = joint_position
        arm_joint_state.joint_velocity = joint_velocity
        arm_joint_state.joint_effort = joint_effort
        arm_joint_state.end_pose = arm_end_pose[:6]

        # Publish motor status
        arm_status = ArmStatus()
        arm_status.header.stamp = self.get_clock().now().to_msg()
        joint_names = self.metal_interface.GetJointNames()
        motor_current = self.metal_interface.GetMotorCurrent()
        rotor_temperature = self.metal_interface.GetRotorTemperature()
        joint_error_code = self.metal_interface.GetJointErrorCode()

        # ROS 2 String array handling
        arm_status.name = [String(data=str(n)) for n in joint_names]
        arm_status.motor_current = motor_current + [sum(motor_current)]
        arm_status.rotor_temperature = rotor_temperature
        arm_status.error_code = joint_error_code

        self.arm_joint_state_pub.publish(arm_joint_state)
        self.arm_status_pub.publish(arm_status)


def main(args=None):
    rclpy.init(args=args)
    controller = MetalController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()


if __name__ == "__main__":
    main()
