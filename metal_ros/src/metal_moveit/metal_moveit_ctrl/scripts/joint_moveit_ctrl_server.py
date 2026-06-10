#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
import math
from geometry_msgs.msg import Pose
from metal_moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlResponse

class JointMoveitCtrlServer:
    def __init__(self):
        rospy.loginfo("Initializing JointMoveitCtrlServer...")

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        available_groups = self.robot.get_group_names()
        rospy.loginfo(f"Available MoveIt groups: {available_groups}")

        self.arm_group = None
        self.gripper_group = None
        self.metal_group = None

        if "arm" in available_groups:
            self.arm_group = moveit_commander.MoveGroupCommander("arm")
            rospy.loginfo("Initialized arm move group.")
        if "gripper" in available_groups:
            self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
            rospy.loginfo("Initialized gripper move group.")
        if "metal" in available_groups:
            self.metal_group = moveit_commander.MoveGroupCommander("metal")
            rospy.loginfo("Initialized metal move group.")

        # Advertise ROS services
        rospy.Service("joint_moveit_ctrl_arm", JointMoveitCtrl, self.handle_arm)
        rospy.Service("joint_moveit_ctrl_gripper", JointMoveitCtrl, self.handle_gripper)
        rospy.Service("joint_moveit_ctrl_metal", JointMoveitCtrl, self.handle_metal)
        rospy.Service("joint_moveit_ctrl_endpose", JointMoveitCtrl, self.handle_endpose)

        rospy.loginfo("Joint MoveIt Control Services Ready.")

    # ---------------------- ARM CONTROL ----------------------
    def handle_arm(self, req):
        rospy.loginfo("Received arm joint movement request.")
        res = JointMoveitCtrlResponse()
        if self.arm_group is None:
            rospy.logerr("Arm move group not initialized.")
            res.status = False
            res.error_code = 1
            return res

        try:
            joint_goal = list(req.joint_states[:6])
            self.arm_group.set_joint_value_target(joint_goal)

            max_vel = max(1e-6, min(1.0 - 1e-6, req.max_velocity))
            max_acc = max(1e-6, min(1.0 - 1e-6, req.max_acceleration))
            self.arm_group.set_max_velocity_scaling_factor(max_vel)
            self.arm_group.set_max_acceleration_scaling_factor(max_acc)

            rospy.loginfo(f"max_velocity: {max_vel} max_acceleration: {max_acc}")
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            res.status = True
            res.error_code = 0
        except Exception as e:
            rospy.logerr(f"Exception during arm movement: {e}")
            res.status = False
            res.error_code = 2
        return res

    # ---------------------- GRIPPER CONTROL ----------------------
    def handle_gripper(self, req):
        rospy.loginfo("Received gripper joint movement request.")
        res = JointMoveitCtrlResponse()
        if self.gripper_group is None:
            rospy.logerr("Gripper move group not initialized.")
            res.status = False
            res.error_code = 1
            return res

        try:
            gripper_goal = [req.gripper]
            self.gripper_group.set_joint_value_target(gripper_goal)
            self.gripper_group.go(wait=True)
            self.gripper_group.stop()
            res.status = True
            res.error_code = 0
        except Exception as e:
            rospy.logerr(f"Exception during gripper movement: {e}")
            res.status = False
            res.error_code = 2
        return res

    # ---------------------- METAL ARM CONTROL ----------------------
    def handle_metal(self, req):
        rospy.loginfo("Received metal joint movement request.")
        res = JointMoveitCtrlResponse()
        if self.metal_group is None:
            rospy.logerr("Metal move group not initialized.")
            res.status = False
            res.error_code = 1
            return res

        try:
            metal_goal = list(req.joint_states[:6]) + [req.gripper]
            self.metal_group.set_joint_value_target(metal_goal)

            max_vel = max(1e-6, min(1.0 - 1e-6, req.max_velocity))
            max_acc = max(1e-6, min(1.0 - 1e-6, req.max_acceleration))
            self.metal_group.set_max_velocity_scaling_factor(max_vel)
            self.metal_group.set_max_acceleration_scaling_factor(max_acc)

            rospy.loginfo(f"max_velocity: {max_vel} max_acceleration: {max_acc}")
            self.metal_group.go(wait=True)
            self.metal_group.stop()
            res.status = True
            res.error_code = 0
        except Exception as e:
            rospy.logerr(f"Exception during metal movement: {e}")
            res.status = False
            res.error_code = 2
        return res

    # ---------------------- END-POSE CONTROL ----------------------
    def handle_endpose(self, req):
        rospy.loginfo("Received endpose movement request.")
        res = JointMoveitCtrlResponse()
        if self.arm_group is None:
            rospy.logerr("Arm move group not initialized.")
            res.status = False
            res.error_code = 1
            return res

        try:
            if len(req.joint_endpose) != 7:
                rospy.logerr("Invalid joint_endpose size. Must be 7 (x,y,z,qx,qy,qz,qw).")
                res.status = False
                res.error_code = 1
                return res

            target_pose = Pose()
            target_pose.position.x = req.joint_endpose[0]
            target_pose.position.y = req.joint_endpose[1]
            target_pose.position.z = req.joint_endpose[2]
            target_pose.orientation.x = req.joint_endpose[3]
            target_pose.orientation.y = req.joint_endpose[4]
            target_pose.orientation.z = req.joint_endpose[5]
            target_pose.orientation.w = req.joint_endpose[6]

            self.arm_group.set_pose_target(target_pose)

            max_vel = max(1e-6, min(1.0 - 1e-6, req.max_velocity))
            max_acc = max(1e-6, min(1.0 - 1e-6, req.max_acceleration))
            self.arm_group.set_max_velocity_scaling_factor(max_vel)
            self.arm_group.set_max_acceleration_scaling_factor(max_acc)

            rospy.loginfo(f"max_velocity: {max_vel} max_acceleration: {max_acc}")
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            res.status = True
            res.error_code = 0
        except Exception as e:
            rospy.logerr(f"Exception during endpose movement: {e}")
            res.status = False
            res.error_code = 2
        return res


def main():
    rospy.init_node("joint_moveit_ctrl_server")
    server = JointMoveitCtrlServer()
    rospy.spin()


if __name__ == "__main__":
    main()
