#pragma once

#include "motor_interface_base/motor_reader_base.h"

namespace makermods {
namespace metal {

class DmMotorReader : public MotorReaderBase {
 public:
  DmMotorReader() = default;

  ~DmMotorReader() = default;

  bool Init(const MotorInfo& motor_info) override;

  bool ReadCanFrame(const can_frame& frame) override;
};

}  // namespace metal
}  // namespace makermods