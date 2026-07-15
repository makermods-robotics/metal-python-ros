#include "metal_sdk_interface.h"

#include <memory>
#include <string>

#include "can_manager.h"

namespace makermods {
namespace metal {

class MetalSDKInterface::Impl {
 public:
  explicit Impl(const std::string& can_id, const std::string& urdf_path,
                int arm_end_type, bool enable_arm)
      : can_manager_(std::make_unique<CanManager>(can_id, urdf_path,
                                                  arm_end_type, enable_arm)) {}
  ~Impl() = default;

  /**
   * @brief must be initialize the SDK interface.
   * @return if the SDK interface is initialized successfully.
   */
  bool Init() { return can_manager_->Init(); }

  /**
   * @brief the interface of arm all joint names.
   * @return 6 or 7(include gripper) joint names.
   */
  std::vector<std::string> GetJointNames() {
    return can_manager_->GetJointNames();
  }

  /**
   * @brief the interface of all motor temperature.
   * @return 6 or 7(include gripper) joint motor temperature.
   */
  std::vector<double> GetRotorTemperature() {
    return can_manager_->GetRotorTemperature();
  }

  /**
   * @brief the interface of all joint error code.
   * @return 6 or 7(include gripper) joint error code.
   */
  std::vector<int> GetJointErrorCode() {
    return can_manager_->GetJointErrorCode();
  }

  /**
   * @brief the interface of all motor current.
   * @return 6 or 7(include gripper) motor current.
   */
  std::vector<double> GetMotorCurrent() {
    return can_manager_->GetMotorCurrent();
  }

  /**
   * @brief the interface of joint position.
   * @return 6 or 7(include gripper) joint position.
   */
  std::vector<double> GetJointPosition() {
    return can_manager_->GetJointPosition();
  }

  /**
   * @brief the interface of joint velocity.
   * @return 6 or 7(include gripper) joint velocity.
   */
  std::vector<double> GetJointVelocity() {
    return can_manager_->GetJointVelocity();
  }

  /**
   * @brief the interface of joint toruqe.
   * @return 6 or 7(include gripper) joint toruqe.
   */
  std::vector<double> GetJointEffort() {
    return can_manager_->GetJointEffort();
  }

  /**
   * @brief the interface of arm end pose.
   * @return 6 size (x y z roll pitch yaw)
   */
  std::array<double, 6> GetArmEndPose() {
    return can_manager_->GetArmEndPose();
  }

  /**
   * @brief set arm control mode. (0: go_zero, 1: gravity compensation)
   */
  void SetArmControlMode(int mode) { can_manager_->SetArmControlMode(mode); }

  /**
   * @brief set arm joint position control command.
   */
  void SetArmJointPosition(const std::array<double, 6>& arm_joint_position,
                           int velocity_ratio) {
    can_manager_->SetArmJointPosition(arm_joint_position, velocity_ratio);
  }

  /**
   * @brief set follow arm joint position control command.
   */
  void SetArmJointPosition(const std::vector<double>& arm_joint_position) {
    can_manager_->SetArmJointPosition(arm_joint_position);
  }

  /**
   * @brief set arm end pose control command. (x y z roll pitch yaw)
   */
  // TODO: std::vector to std::array?
  void SetArmEndPose(const std::array<double, 6>& arm_end_pose) {
    can_manager_->SetArmEndPose(arm_end_pose);
  }

  /**
   * @brief set gripper stroke control command. Unit: mm
   */
  void SetGripperStroke(double gripper_stroke, int velocity_ratio) {
    can_manager_->SetGripperStroke(gripper_stroke, velocity_ratio);
  }

  /**
   * @brief Enable or disable the motor of the robot arm.
   * @param enable_arm true: enable the motor, false: disable the motor.
   */
  void SetEnableArm(bool enable_flag) {
    can_manager_->SetEnableArm(enable_flag);
  }

  /**
   * @brief Save J6 joint zero position.
   */
  void SaveJ6ZeroPosition() { can_manager_->SaveJ6ZeroPosition(); }

 private:
  std::unique_ptr<CanManager> can_manager_;
};

MetalSDKInterface::MetalSDKInterface(const std::string& can_id,
                               const std::string& urdf_path, int arm_end_type,
                               bool enable_arm)
    : pimpl_(std::make_unique<Impl>(can_id, urdf_path, arm_end_type,
                                    enable_arm)) {}

MetalSDKInterface::~MetalSDKInterface() = default;

bool MetalSDKInterface::Init() {
  if (!pimpl_->Init()) {
    return false;
  }

  return true;
}

std::vector<std::string> MetalSDKInterface::GetJointNames() {
  return pimpl_->GetJointNames();
}

std::vector<double> MetalSDKInterface::GetRotorTemperature() {
  return pimpl_->GetRotorTemperature();
}

std::vector<int> MetalSDKInterface::GetJointErrorCode() {
  return pimpl_->GetJointErrorCode();
}

std::vector<double> MetalSDKInterface::GetMotorCurrent() {
  return pimpl_->GetMotorCurrent();
}

std::vector<double> MetalSDKInterface::GetJointPosition() {
  return pimpl_->GetJointPosition();
}

std::vector<double> MetalSDKInterface::GetJointVelocity() {
  return pimpl_->GetJointVelocity();
}

std::vector<double> MetalSDKInterface::GetJointEffort() {
  return pimpl_->GetJointEffort();
}

std::array<double, 6> MetalSDKInterface::GetArmEndPose() {
  return pimpl_->GetArmEndPose();
}

void MetalSDKInterface::SetArmControlMode(const ControlMode& mode) {
  pimpl_->SetArmControlMode(static_cast<int>(mode));
}

void MetalSDKInterface::SetArmJointPosition(
    const std::array<double, 6>& arm_joint_position, int velocity_ratio) {
  pimpl_->SetArmJointPosition(arm_joint_position, velocity_ratio);
}

/**
 * @brief set follow arm joint position control command.
 */
void MetalSDKInterface::SetArmJointPosition(
    const std::vector<double>& arm_joint_position) {
  pimpl_->SetArmJointPosition(arm_joint_position);
}

void MetalSDKInterface::SetArmEndPose(const std::array<double, 6>& arm_end_pose) {
  pimpl_->SetArmEndPose(arm_end_pose);
}

void MetalSDKInterface::SetGripperStroke(double gripper_stroke,
                                      int velocity_ratio) {
  pimpl_->SetGripperStroke(gripper_stroke, velocity_ratio);
}

void MetalSDKInterface::SetEnableArm(bool enable_flag) {
  pimpl_->SetEnableArm(enable_flag);
}

void MetalSDKInterface::SaveJ6ZeroPosition() { pimpl_->SaveJ6ZeroPosition(); }

}  // namespace metal
}  // namespace makermods