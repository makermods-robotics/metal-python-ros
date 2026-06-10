#!/usr/bin/env python3
import rospy, rospkg
from metal_msg.msg import ArmStatus, ArmJointState, ArmEndPoseControl, ArmJointPositionControl
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from metal_sdk import MetalSDKInterface, ControlMode

class MetalController:
    def __init__(self):
        rospy.init_node("metal_controller_node", anonymous=True)

        # ROS parameters
        self.can_id = rospy.get_param("~arm_can_id", "can0")
        self.arm_feedback_rate = rospy.get_param("~arm_feedback_rate", 200)
        self.arm_end_pose_control_topic = rospy.get_param(
            "~arm_end_pose_control_topic", "/metal/arm_end_pose_control"
        )
        self.arm_joint_position_control_topic = rospy.get_param(
            "~arm_joint_position_control_topic", "/metal/arm_joint_position_control_topic"
        )
        self.arm_joint_state_topic = rospy.get_param(
            "~arm_joint_state_topic", "/metal/arm_joint_state"
        )
        self.arm_status_topic = rospy.get_param(
            "~arm_status_topic", "/metal/arm_status"
        )
        self.sim_joint_postion_control_topic = rospy.get_param(
            "~sim_joint_postion_control_topic", "/joint_states"
        )
        self.is_sim = rospy.get_param("~is_sim", False)
        self.arm_control_type = rospy.get_param("~arm_control_type", "follower_arm")
        self.arm_end_type = rospy.get_param("~arm_end_type", 0)
        self.auto_enable = rospy.get_param("~auto_enable", True)

        # URDF path
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("metal_controller")

        if self.arm_end_type == 0:
            urdf_path = f"{package_path}/urdf/metal_no_gripper.urdf"
        elif self.arm_end_type == 1:
            urdf_path = f"{package_path}/urdf/metal_with_gripper.urdf"
        elif self.arm_end_type == 2:
            urdf_path = f"{package_path}/urdf/metal_with_gripper.urdf"
        elif self.arm_end_type == 3:
            urdf_path = f"{package_path}/urdf/metal_with_gripper.urdf"
        else:
            rospy.logerr(f"arm_end_type {self.arm_end_type} not supported")
            raise RuntimeError("Unsupported arm_end_type")

        # Initialize the Metal SDK
        self.metal_interface = MetalSDKInterface(
            can_id=self.can_id,
            urdf_path=urdf_path,
            arm_end_type=self.arm_end_type,
            enable_arm=self.auto_enable,
        )
        if not self.metal_interface.Init():
            rospy.logerr("Init Metal SDK interface failed")
            raise RuntimeError("Metal SDK init failed")

        # Set control mode
        if self.arm_control_type == "leader_arm":
            self.metal_interface.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
        elif self.arm_control_type == "follower_arm":
            self.metal_interface.SetArmControlMode(ControlMode.RT_JOINT_POSITION)
            self.arm_end_pose_sub = rospy.Subscriber(
                self.arm_end_pose_control_topic, ArmEndPoseControl, self.arm_end_pose_callback
            )
            self.arm_joint_pos_sub = rospy.Subscriber(
                self.arm_joint_position_control_topic, ArmJointState, self.follow_arm_joint_callback
            )
        elif self.arm_control_type == "normal_arm":
            self.metal_interface.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
            self.arm_end_pose_sub = rospy.Subscriber(
                self.arm_end_pose_control_topic, ArmEndPoseControl, self.arm_end_pose_callback
            )
            self.arm_joint_pos_sub = rospy.Subscriber(
                self.arm_joint_position_control_topic, ArmJointPositionControl, self.arm_joint_position_callback
            )
            if self.is_sim:
                self.arm_joint_pos_sub = rospy.Subscriber(
                   self.sim_joint_postion_control_topic, JointState, self.sim_joint_position_callback
                )
        else:
            rospy.logerr(f"arm_control_type {self.arm_control_type} not supported")
            raise RuntimeError("Unsupported arm_control_type")

        # Publishers
        self.arm_joint_state_pub = rospy.Publisher(self.arm_joint_state_topic, ArmJointState, queue_size=1)
        self.arm_status_pub = rospy.Publisher(self.arm_status_topic, ArmStatus, queue_size=1)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.arm_feedback_rate), self.arm_information_timer_callback)

        rospy.loginfo("Metal controller initialized successfully!")

    # ---------------- Callbacks ----------------
    def arm_end_pose_callback(self, msg: ArmEndPoseControl):
        arm_end_pose = list(msg.end_pose[:6])
        self.metal_interface.SetArmEndPose(arm_end_pose)
        self.metal_interface.SetGripperStroke(msg.gripper_stroke, msg.gripper_velocity)

    def follow_arm_joint_callback(self, msg: ArmJointState):
        if len(msg.joint_position) >= 6:
            self.metal_interface.SetFollowerArmJointPosition(msg.joint_position)
        else:
            rospy.logerr("follow arm receive joint control size < 6")

    def arm_joint_position_callback(self, msg: ArmJointPositionControl):
        # control J1 - J6 joint
        arm_joint_position = list(msg.joint_position[:6])
        self.metal_interface.SetArmJointPosition(arm_joint_position, msg.joint_velocity)
        # control gripper
        self.metal_interface.SetGripperStroke(msg.gripper_stroke, msg.gripper_velocity)

    def sim_joint_position_callback(self, msg: JointState):
        # control J1 - J6 joint
        arm_joint_position = list(msg.position[:6])
        self.metal_interface.SetArmJointPosition(arm_joint_position, 6)
        # control gripper
        if len(msg.position) >= 7:
            self.metal_interface.SetGripperStroke(-msg.position[6] * 2000, 6)

    def arm_information_timer_callback(self, event):
        # Publish joint state
        arm_joint_state = ArmJointState()
        arm_joint_state.header.stamp = rospy.Time.now()

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
        arm_status.header.stamp = rospy.Time.now()
        joint_names = self.metal_interface.GetJointNames()
        motor_current = self.metal_interface.GetMotorCurrent()
        rotor_temperature = self.metal_interface.GetRotorTemperature()
        joint_error_code = self.metal_interface.GetJointErrorCode()

        arm_status.name = [String(data=n) for n in joint_names]
        arm_status.motor_current = motor_current + [sum(motor_current)]
        arm_status.rotor_temperature = rotor_temperature
        arm_status.error_code = joint_error_code

        self.arm_joint_state_pub.publish(arm_joint_state)
        self.arm_status_pub.publish(arm_status)


if __name__ == "__main__":
    try:
        controller = MetalController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
