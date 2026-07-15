#include "motor_interface_base/motor_writer_base.h"

namespace makermods {
namespace metal {

bool MotorWriterBase::Init(const MotorInfo& motor_info) {
  motor_info_ = motor_info;

  return true;
}

}  // namespace metal
}  // namespace makermods