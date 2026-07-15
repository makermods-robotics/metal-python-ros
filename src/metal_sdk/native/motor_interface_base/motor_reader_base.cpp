#include "motor_interface_base/motor_reader_base.h"
#include <cmath>

namespace makermods {
namespace metal {

bool MotorReaderBase::Init(const MotorInfo& motor_info) {
  motor_info_ = motor_info;
  motor_state_.name = motor_info_.joint_name;

  return true;
}

bool MotorReaderBase::IsDisEnable() {
  return motor_state_.status == Status::Disabled;
}

}  // namespace metal
}  // namespace makermods