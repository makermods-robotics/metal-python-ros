#!/usr/bin/env python3
"""Star Arm 102 (reBot Arm 102) leader node for Metal follower teleoperation.

Reads the FashionStar UART smart servos of a Seeed Studio Star Arm 102 leader and
publishes ArmJointState commands that a `metal_controller` node running as
`follower_arm` consumes on its `arm_joint_position_control_topic`.

Message convention (matches the Metal SDK follower):
  joint_position = [J1..J6 in radians, gripper stroke in mm (0..80)]

The per-joint mapping (servo ids, direction/scale, output ranges in degrees) mirrors
the LeRobot `rebot_102_leader` + `tools/star_metal.json` calibration. Servo zero
points are stored in the servos themselves (set once with the LeRobot calibration
flow or the vendor tool), so this node has no calibration file.

On startup the node performs a slow sync: commands ramp from the follower's current
pose to the leader pose, so the follower never snaps across a large gap.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from metal_msg.msg import ArmJointState

try:
    from motorbridge_smart_servo import FashionStarServo
except ImportError as e:  # pragma: no cover - import guard for a pip-only dependency
    raise ImportError(
        "motorbridge_smart_servo is required for the Star Arm 102 leader. "
        "Install it with: pip install 'motorbridge-smart-servo>=0.0.4,<0.1.0'"
    ) from e

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
GRIPPER_IDX = 6
GRIPPER_MAX_MM = 80.0

# Nonlinear gripper stroke(mm) <-> motor angle(rad) lookup, ported verbatim from the
# Metal SDK (can_manager.h distances_/angles_). NOT a linear gear ratio.
_DIST = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
    48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93,
    94, 95, 96, 97, 98, 99, 100, 102.5,
]
_ANG = [
    0.002, 0.01407, 0.0368934, 0.08634, 0.115854, 0.161084, 0.176609, 0.194816, 0.22893,
    0.262277, 0.295, 0.314215, 0.344305, 0.362704, 0.393177, 0.411575, 0.441473, 0.46773,
    0.482871, 0.508743, 0.527718, 0.550333, 0.576397, 0.60227, 0.621053, 0.639452, 0.662833,
    0.683915, 0.707105, 0.722054, 0.748693, 0.763451, 0.782233, 0.805039, 0.815964, 0.841837,
    0.856786, 0.879593, 0.894158, 0.91639, 0.92789, 0.94648, 0.969287, 0.987494, 0.998418,
    1.02084, 1.03617, 1.05112, 1.0701, 1.08505, 1.10383, 1.11897, 1.13411, 1.15557, 1.16382,
    1.18221, 1.20117, 1.21594, 1.23779, 1.24986, 1.26424, 1.28264, 1.29452, 1.31273, 1.3317,
    1.34359, 1.36505, 1.38019, 1.39169, 1.40683, 1.42542, 1.44401, 1.45609, 1.47468, 1.48943,
    1.50419, 1.52259, 1.53735, 1.55575, 1.57453, 1.582, 1.60462, 1.62302, 1.64218, 1.65694,
    1.66825, 1.68722, 1.70984, 1.729, 1.73973, 1.76216, 1.78477, 1.79991, 1.82214, 1.84399,
    1.86297, 1.88577, 1.9036, 1.92717, 1.94844, 1.97527, 2.03143,
]


def _rad_to_stroke_mm(rad: float) -> float:
    if rad <= _ANG[0]:
        return float(_DIST[0])
    if rad >= _ANG[-1]:
        return float(_DIST[-1])
    for i in range(1, len(_ANG)):
        if rad < _ANG[i]:
            nearest = _DIST[i - 1] if abs(rad - _ANG[i - 1]) < abs(_ANG[i] - rad) else _DIST[i]
            return float(nearest)
    return float(_DIST[-1])


class StarArmLeader(Node):
    def __init__(self):
        super().__init__("star_arm_leader_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("publish_rate", 100.0)
        # Ordered [joint1..joint6, gripper]; directions/ranges follow the LeRobot
        # star_metal calibration (degrees, leader-side).
        self.declare_parameter("joint_ids", [0, 1, 2, 3, 4, 5, 6])
        self.declare_parameter("joint_directions", [-1.0, -1.0, -0.667, -1.0, 1.0, -1.0, 1.895])
        self.declare_parameter("joint_range_min", [-160.0, -180.0, 0.0, -123.0, -85.0, -145.0, -5.0])
        self.declare_parameter("joint_range_max", [160.0, 0.0, 180.0, 81.0, 85.0, 145.0, 115.0])
        # Command topic consumed by the follower node, and the follower's own joint
        # state feedback (used to start the slow sync from the follower's real pose).
        self.declare_parameter("arm_joint_state_topic", "/master_arm_right/joint_states")
        self.declare_parameter("follower_joint_state_topic", "/puppet_arm_right/joint_states")
        # Per-tick command slew limits during startup sync, and the convergence
        # tolerances that end it (full-speed tracking afterwards).
        self.declare_parameter("startup_sync_step_deg", 0.6)
        self.declare_parameter("startup_sync_step_mm", 1.0)
        self.declare_parameter("startup_sync_tolerance_deg", 3.0)
        self.declare_parameter("startup_sync_tolerance_mm", 5.0)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.joint_ids = list(self.get_parameter("joint_ids").value)
        self.joint_directions = list(self.get_parameter("joint_directions").value)
        self.range_min = list(self.get_parameter("joint_range_min").value)
        self.range_max = list(self.get_parameter("joint_range_max").value)
        self.sync_step_deg = self.get_parameter("startup_sync_step_deg").value
        self.sync_step_mm = self.get_parameter("startup_sync_step_mm").value
        self.sync_tol_deg = self.get_parameter("startup_sync_tolerance_deg").value
        self.sync_tol_mm = self.get_parameter("startup_sync_tolerance_mm").value

        n = len(JOINT_NAMES)
        if not (len(self.joint_ids) == len(self.joint_directions) == len(self.range_min) == len(self.range_max) == n):
            raise RuntimeError(f"joint_ids/joint_directions/joint_range_min/joint_range_max must all have {n} entries")

        self.bus = FashionStarServo(self.port, baudrate=self.baudrate)
        for name, servo_id in zip(JOINT_NAMES, self.joint_ids):
            if not self.bus.ping(servo_id):
                self.bus.close()
                raise RuntimeError(f"Star Arm servo not found for {name} (id={servo_id}) on {self.port}")
        for servo_id in self.joint_ids:
            self.bus.unlock(servo_id)
        for servo_id in self.joint_ids:
            self.bus.reset_multi_turn(servo_id)

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(
            ArmJointState, self.get_parameter("arm_joint_state_topic").value, qos_profile
        )
        self.follower_sub = self.create_subscription(
            ArmJointState,
            self.get_parameter("follower_joint_state_topic").value,
            self.follower_state_callback,
            qos_profile,
        )

        # Command state: [J1..J6 rad, gripper mm]. Seeded from the follower's first
        # feedback message, then slewed toward the leader pose until synced.
        self.cmd = None
        self.synced = False
        self.last_positions = None

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f"Star Arm 102 leader initialized on {self.port}!")

    def follower_state_callback(self, msg: ArmJointState):
        if self.cmd is None and len(msg.joint_position) >= 7:
            self.cmd = list(msg.joint_position[:7])
            self.get_logger().info("Follower pose received; starting slow sync to leader.")
        # Only the first message is needed; afterwards the command state evolves on its own.

    @staticmethod
    def _unwrap(value: float, min_value: float, max_value: float) -> float:
        """Unwrap a multi-turn servo angle into the ±180° window centred on (min+max)/2."""
        center = (min_value + max_value) / 2.0
        turns = round((value - center) / 360.0)
        return value - turns * 360.0

    def read_leader_targets(self):
        """Read the servos and return targets [J1..J6 rad, gripper mm]."""
        result = self.bus.sync_monitor(self.joint_ids)
        targets = []
        for i, (name, servo_id) in enumerate(zip(JOINT_NAMES, self.joint_ids)):
            monitor = result.get(servo_id)
            if monitor is None:
                raise RuntimeError(f"Servo {name} (id={servo_id}) did not respond")
            direction = self.joint_directions[i]
            sign = 1.0 if direction >= 0 else -1.0
            unwrapped = self._unwrap(monitor.angle_deg, self.range_min[i] * sign, self.range_max[i] * sign)
            pos_deg = max(self.range_min[i], min(self.range_max[i], unwrapped * direction))
            if i == GRIPPER_IDX:
                # Leader gripper degrees -> motor rad -> stroke mm via the vendor table.
                stroke = _rad_to_stroke_mm(math.radians(max(0.0, pos_deg)))
                targets.append(max(0.0, min(GRIPPER_MAX_MM, stroke)))
            else:
                targets.append(math.radians(pos_deg))
        return targets

    def timer_callback(self):
        try:
            targets = self.read_leader_targets()
            self.last_positions = targets
        except Exception as e:
            if self.last_positions is None:
                self.get_logger().error(f"Failed to read Star Arm servos: {e}", throttle_duration_sec=1.0)
                return
            self.get_logger().error(
                f"Failed to read Star Arm servos, repeating last command: {e}", throttle_duration_sec=1.0
            )
            targets = self.last_positions

        if self.cmd is None:
            self.get_logger().warn(
                "Waiting for follower joint state before commanding (is the follower node running?)",
                throttle_duration_sec=2.0,
            )
            return

        if not self.synced:
            step_rad = math.radians(self.sync_step_deg)
            max_err_rad, max_err_mm = 0.0, 0.0
            for i in range(len(targets)):
                err = targets[i] - self.cmd[i]
                if i == GRIPPER_IDX:
                    max_err_mm = max(max_err_mm, abs(err))
                    self.cmd[i] += max(-self.sync_step_mm, min(self.sync_step_mm, err))
                else:
                    max_err_rad = max(max_err_rad, abs(err))
                    self.cmd[i] += max(-step_rad, min(step_rad, err))
            if max_err_rad <= math.radians(self.sync_tol_deg) and max_err_mm <= self.sync_tol_mm:
                self.synced = True
                self.get_logger().info("Follower synced to leader; tracking at full speed.")
        else:
            self.cmd = list(targets)

        msg = ArmJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_position = [float(v) for v in self.cmd]
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StarArmLeader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
