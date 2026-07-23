#include "motor_readers/dm_motor_reader.h"

#include <sys/types.h>

#include <cstdint>

#include "common/log.h"

namespace makermods {
namespace metal {

bool DmMotorReader::Init(const MotorInfo& motor_info) {
  if (!MotorReaderBase::Init(motor_info)) {
    AERROR << "Failed to CanReaderBase::Init";
    return false;
  }

  return true;
}

bool DmMotorReader::ReadCanFrame(const can_frame& frame) {
  static auto uint_to_float = [](uint16_t x, float xmin, float xmax,
                                 uint8_t bits) -> float {
    float span = xmax - xmin;
    float data_norm = float(x) / ((1 << bits) - 1);
    float data = data_norm * span + xmin;

    return data;
  };

  if (frame.can_id != motor_info_.motor_read_info.id) {
    AERROR << "can id not matched";
    return false;
  }

  uint32_t canID = frame.can_id;
  if (read_write_save) {
    // if (frame.data[2] == 0x33 || frame.data[2] == 0x55 ||
    //     frame.data[2] == 0xAA) {
    //   //发的是读参数或写参数命令，返回对应寄存器参数
    //   if (frame.data[2] == 0x33 || frame.data[2] == 0x55) {
    //     //写参数或者读参数返回
    //     receive_param(&frame.data[0]);
    //     read_write_save = false;
    //   }
    // }
    read_write_save = false;
  } else {
    auto frame_data = frame.data;
    // error code
    uint8_t error_code = frame_data[0] >> 4;
    switch (error_code) {
      case 0x0:
        motor_state_.status = Status::Disabled;
        break;
      case 0x1:
        motor_state_.status = Status::Enabled;
        break;
      case 0x8:
        motor_state_.status = Status::OverVoltage;
        break;
      case 0x9:
        motor_state_.status = Status::UnderVoltage;
        break;
      case 0xA:
        motor_state_.status = Status::Overcurrent;
        break;
      case 0xB:
        motor_state_.status = Status::MosOverTemperature;
        break;
      case 0xC:
        motor_state_.status = Status::RotorOverTemperature;
        break;
      case 0xD:
        motor_state_.status = Status::MotorDisconnected;
        break;
      case 0xE:
        motor_state_.status = Status::Overload;
        break;
      // 若有其他你想特别处理的 err 值，可以继续添加 case
      default:
        AWARN << "motor unkonwn status.";
    }

    // position、velocity、torque
    uint16_t p_int = (uint16_t(frame_data[1]) << 8) | frame_data[2];
    uint16_t v_int = (uint16_t(frame_data[3]) << 4) | (frame_data[4] >> 4);
    uint16_t t_int = (uint16_t(frame_data[4] & 0xF) << 8) | frame_data[5];
    // convert
    double pmax = motor_info_.pmax;
    double vmax = motor_info_.vmax;
    double tmax = motor_info_.tmax;
    motor_state_.position = uint_to_float(p_int, -pmax, pmax, 16);
    motor_state_.velocity = uint_to_float(v_int, -vmax, vmax, 12);
    motor_state_.torque = uint_to_float(t_int, -tmax, tmax, 12);

    // compute current
    // 扭矩系数=1.5*极对数*磁链*减速比*齿轮系数，前提是KT_OUT设置为0就采用公式自动计算扭矩系数
    // 扭矩 = 相电流*扭矩系数
    float torque_effocient =
        1.5 * motor_info_.pole_pairs * motor_info_.magnet_link *
        motor_info_.reduction_ratio * motor_info_.gear_coefficient;
    motor_state_.current = motor_state_.torque / torque_effocient;

    // t_mos、and t_rotor
    uint8_t t_mos = frame_data[6];
    uint8_t t_rotor = frame_data[7];
    motor_state_.mos_temperature = t_mos;
    motor_state_.rotor_temperature = t_rotor;
  }

  return true;
}

}  // namespace metal
}  // namespace makermods