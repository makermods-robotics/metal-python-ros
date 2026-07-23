#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace makermods {
namespace metal {

class MetalSDKInterface {
 public:
  MetalSDKInterface() = delete;
  explicit MetalSDKInterface(const std::string& can_id,
                          const std::string& urdf_path, int arm_end_type,
                          bool enable_arm);
  ~MetalSDKInterface();

  /**
   * @brief must be initialize the SDK interface.
   * @return if the SDK interface is initialized successfully.
   */
  bool Init();

  enum ControlMode {
    GRAVITY_COMPENSATION = 0,
    RT_JOINT_POSITION = 1,
    NRT_JOINT_POSITION
  };

  /**
   * @brief the interface of arm all joint names.
   * @return 6 or 7(include gripper) joint names.
   */
  std::vector<std::string> GetJointNames();

  /**
   * @brief the interface of all motor internal coil temperature
   * @return 6 or 7(include gripper) joint rotor temperature.
   */
  std::vector<double> GetRotorTemperature();

  /**
   * @brief the interface of all joint error code.
   * 0: Disabled, 1: Enabled, 2: MotorDisconnected, 3: OverVoltage, 4:
   * UnderVoltage 5: Overcurrent, 6: MosOverTemperature, 7:
   * RotorOverTemperature, 8: Overload
   * @return 6 or 7(include gripper) joint error code.
   */
  std::vector<int> GetJointErrorCode();

  /**
   * @brief the interface of all motor current.
   * @return 6 or 7(include gripper) motor current.
   */
  std::vector<double> GetMotorCurrent();

  /**
   * @brief the interface of joint position.
   * @return 6 or 7(include gripper) joint position.
   */
  std::vector<double> GetJointPosition();

  /**
   * @brief the interface of joint velocity.
   * @return 6 or 7(include gripper) joint velocity.
   */
  std::vector<double> GetJointVelocity();

  /**
   * @brief the interface of joint toruqe.
   * @return 6 or 7(include gripper) joint toruqe.
   */
  std::vector<double> GetJointEffort();

  /**
   * @brief the interface of arm end pose.
   * @return 6 size (x y z roll pitch yaw)
   */
  std::array<double, 6> GetArmEndPose();

  /**
   * @brief compute the model gravity torque G(q) at an ARBITRARY joint pose,
   * without energizing the arm. For gravity-comp calibration / direction check.
   * @param joint_position 6 joint angles (rad), base->tip (J1..J6).
   * @return 6 gravity torques (Nm) predicted by the URDF model.
   */
  std::vector<double> ComputeGravityTorque(
      const std::vector<double>& joint_position);

  /**
   * @brief set arm control mode. (0: GRAVITY_COMPENSATION, 1:
   * RT_JOINT_POSITION, 2: NRT_JOINT_POSITION)
   */
  void SetArmControlMode(const ControlMode& mode);

  /**
   * @brief set normal control arm joint position and velocity ratio.
   * @param arm_joint_position J1 - J6 joint position
   * @param velocity_ratio range:[1, 10], default is 5, 1 is slower, 10 is
   * faster.
   */
  void SetArmJointPosition(const std::array<double, 6>& arm_joint_position,
                           int velocity_ratio = 5);

  /**
   * @brief set follow arm joint position control command.
   * @param arm_joint_position all joint position, include gripper
   */
  void SetArmJointPosition(const std::vector<double>& arm_joint_position);

  /**
   * @brief set arm end pose control command. (x y z roll pitch yaw)
   */
  void SetArmEndPose(const std::array<double, 6>& arm_end_pose);

  /**
   * @brief set gripper stroke control command. Unit: mm
   * @param gripper_stroke Unit: mm, range: [0, 80]
   * @param velocity_ratio range:[1, 10], default is 5, 1 is slower, 10 is
   * faster.
   */
  void SetGripperStroke(double gripper_stroke, int velocity_ratio = 5);

  /**
   * @brief Enable or disable all joint motor.
   * @param enable_arm true: enable, false: disable.
   */
  void SetEnableArm(bool enable_flag);

  /**
   * @brief Save J6 joint zero position.
      Each time you reinstall the end flange adapter or gripper, you need to
   set the zero point of J6.
  */
  void SaveJ6ZeroPosition();

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace metal
}  // namespace makermods