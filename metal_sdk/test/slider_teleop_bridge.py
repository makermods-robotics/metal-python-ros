#!/usr/bin/env python3
"""Slider -> real arm teleop bridge (DELTA / relative, jump-safe).

jsp_gui sliders publish /joint_states starting from ~0. Commanding those
ABSOLUTE values would snap the arm from its current pose to zero — dangerous.
Instead this bridge is RELATIVE:

  target[i] = q0[i] + clamp(slider[i] - s0[i], +/-MAX_DELTA)

where q0 = the arm's pose when teleop starts, s0 = the sliders' first snapshot.
So the arm starts exactly where it is, and each slider moves its joint by the
same amount you move the slider — no jump. Per-joint travel is capped and speed
is the slowest setting.

Publishes metal_msg/ArmJointPositionControl to the driver's control topic
(driver must run: arm_control_type=normal_arm, auto_enable=true).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from metal_msg.msg import ArmJointState, ArmJointPositionControl

EXPECTED = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MAX_DELTA = 0.6   # rad: hard cap on how far each joint may travel from q0 (safety)
VEL = 1           # velocity ratio 1..10, 1 = slowest


class SliderTeleop(Node):
    def __init__(self):
        super().__init__("slider_teleop_bridge")
        topic = self.declare_parameter(
            "control_topic", "/metal/arm_joint_position_control_topic").value
        self.q0 = None   # arm pose at start (list of 6)
        self.s0 = None   # slider snapshot at start (list of 6)
        self.pub = self.create_publisher(ArmJointPositionControl, topic, 10)
        self.create_subscription(ArmJointState, "/metal/arm_joint_state", self.on_state, 10)
        self.create_subscription(JointState, "/joint_states", self.on_slider, 10)
        self.get_logger().info(
            "slider teleop -> %s | waiting for arm state + sliders (MAX_DELTA=%.2f rad)"
            % (topic, MAX_DELTA))

    def on_state(self, msg):
        if self.q0 is None and len(msg.joint_position) >= 6:
            self.q0 = [float(x) for x in msg.joint_position[:6]]
            self.get_logger().info("captured arm start q0=%s" %
                                   [round(x, 3) for x in self.q0])

    def on_slider(self, msg):
        m = {n: p for n, p in zip(msg.name, msg.position)}
        if not all(n in m for n in EXPECTED):
            return
        s = [float(m[n]) for n in EXPECTED]
        if self.s0 is None:
            self.s0 = s
            self.get_logger().info("captured slider start s0=%s" %
                                   [round(x, 3) for x in s])
            return
        if self.q0 is None:
            return
        target = []
        for i in range(6):
            d = s[i] - self.s0[i]
            if d > MAX_DELTA:
                d = MAX_DELTA
            elif d < -MAX_DELTA:
                d = -MAX_DELTA
            target.append(self.q0[i] + d)
        out = ArmJointPositionControl()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.joint_position = target
        out.joint_velocity = VEL
        out.gripper_stroke = 0.0
        out.gripper_velocity = VEL
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SliderTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
