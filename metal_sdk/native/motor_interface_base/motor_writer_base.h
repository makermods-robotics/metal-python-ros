#pragma once

#include <linux/can.h>

#include <string>

// #include "common/motor_control_command.h"
#include "common/motor_state.h"
#include "config/motor_config.h"

namespace makermods {
namespace metal {

class MotorWriterBase {
 public:
  MotorWriterBase() = default;

  virtual ~MotorWriterBase() = default;

  virtual bool Init(const MotorInfo& motor_info);

  /**
   * @brief Mit Control Mode
   */
  virtual void MitControl(can_frame& frame,
                          const ControlCommand& control_command,
                          bool need_position_limit = true) = 0;
  /**
   * @brief Position Velocity Control Mode
   */
  virtual void PosVelControl(can_frame& frame,
                             const ControlCommand& control_command,
                             bool need_position_limit = true) = 0;

  /**
   * * @brief return enable motor can frame
   */
  virtual void Enable(can_frame& frame) = 0;

  /**
   * * @brief return disable motor can frame
   */
  virtual void Disable(can_frame& frame) = 0;

  /**
   * * @brief return set zero joint position can frame
   */
  virtual void SetZeroPosition(can_frame& frame) = 0;

  /**
   * @brief switch control mode.
   */
  virtual void SwitchControlMode(const ControlMode& mode, can_frame& frame) = 0;

  /**
   * @brief change the position kp of position velocity mode.
   */
  virtual void SetPositionKp(can_frame& frame, float kp) = 0;

  /**
   * @brief change the velocity kp of position velocity mode.
   */
  virtual void SetVelocityKd(can_frame& frame, float kd) = 0;

  /**
   * @brief change the current bandwidth.
   */
  virtual void SetCurrentBW(can_frame& frame, float ibw) = 0;

  /**
   * @brief change the current bandwidth.
   */
  virtual void SetSpeedEnhanceFactor(can_frame& frame, float factor) = 0;

  float MitKp() { return motor_info_.mit_kp; }

  float MitKd() { return motor_info_.mit_kd; }

  float FollowMitKp() { return motor_info_.follow_mit_kp; }

  float FollowMitKd() { return motor_info_.follow_mit_kd; }

  float Ibw() { return motor_info_.ibw; }

  float SpeedEnhanceFactor() { return motor_info_.speed_enhance_factor; }

  float PosKp() { return motor_info_.pos_kp; }

  float VelKp() { return motor_info_.vel_kp; }

  std::string name() { return motor_info_.joint_name; }

 protected:
  MotorInfo motor_info_;
};

}  // namespace metal
}  // namespace makermods