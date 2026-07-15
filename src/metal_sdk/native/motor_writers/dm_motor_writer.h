#pragma once

#include <linux/can.h>

#include <cstdint>

#include "motor_interface_base/motor_writer_base.h"

namespace makermods {
namespace metal {

class DmMotorWriter : public MotorWriterBase {
 public:
  DmMotorWriter() = default;

  ~DmMotorWriter() = default;

  bool Init(const MotorInfo& motor_info) override;

  void MitControl(can_frame& frame, const ControlCommand& control_command,
                  bool need_position_limit = true) override;

  /**
   * @brief Position Velocity Control Mode
   */
  void PosVelControl(can_frame& frame, const ControlCommand& control_command,
                     bool need_position_limit = true) override;

  void Enable(can_frame& frame) override;

  void Disable(can_frame& frame) override;

  void SetZeroPosition(can_frame& frame) override;

  /**
   * @brief switch control mode.
   */
  void SwitchControlMode(const ControlMode& mode, can_frame& frame) override;

  /**
   * @brief change the position kp of position velocity mode.
   */
  void SetPositionKp(can_frame& frame, float kp) override;

  /**
   * @brief change the velocity kp of position velocity mode.
   */
  void SetVelocityKd(can_frame& frame, float kd) override;

  /**
   * @brief change the current bandwidth.
   */
  void SetCurrentBW(can_frame& frame, float ibw) override;

  void SetSpeedEnhanceFactor(can_frame& frame, float factor) override;

 private:
  can_frame WriteMotorParam(uint8_t rid, uint8_t data[4]) const;

  can_frame ReadMotorParam(uint8_t rid) const;
};

}  // namespace metal
}  // namespace makermods