#include "motor_writers/dm_motor_writer.h"

#include <array>
#include <cmath>

#include "common/log.h"

namespace makermods {
namespace metal {

bool DmMotorWriter::Init(const MotorInfo& motor_info) {
  if (!MotorWriterBase::Init(motor_info)) {
    AERROR << "Failed to CanWriterBase::Init";
    return false;
  }

  AINFO << name() << " min position limit: " << motor_info_.position_min
        << " , max position limit: " << motor_info_.position_max;

  return true;
}

void DmMotorWriter::MitControl(can_frame& frame,
                               const ControlCommand& control_command,
                               bool need_position_limit) {
  static auto float_to_uint = [](float x, float xmin, float xmax,
                                 uint8_t bits) -> uint16_t {
    float span = xmax - xmin;
    float data_norm = (x - xmin) / span;
    uint16_t data_uint = data_norm * ((1 << bits) - 1);
    return data_uint;
  };

  // NOTE(known-limitation): joint soft limit only applied when
  // need_position_limit is set by the caller; not enforced unconditionally
  // here. 见 known-limitations 文档
  float position_limit = control_command.position;
  if (need_position_limit) {
    position_limit =
        std::max(motor_info_.position_min,
                 std::min(position_limit, motor_info_.position_max));
  }

  float torque_limit = std::max(
      -motor_info_.tmax, std::min(control_command.torque, motor_info_.tmax));

  float pmax = motor_info_.pmax;
  float vmax = motor_info_.vmax;
  float tmax = motor_info_.tmax;
  uint16_t pos_tmp = float_to_uint(position_limit, -pmax, pmax, 16);
  uint16_t vel_tmp = float_to_uint(control_command.velocity, -vmax, vmax, 12);
  uint16_t kp_tmp = float_to_uint(control_command.kp, 0, 500, 12);
  uint16_t kd_tmp = float_to_uint(control_command.kd, 0, 5, 12);
  uint16_t tor_tmp = float_to_uint(torque_limit, -tmax, tmax, 12);

  frame.can_id = motor_info_.motor_write_info.id;
  frame.can_dlc = 8;
  frame.data[0] = (pos_tmp >> 8);
  frame.data[1] = pos_tmp;
  frame.data[2] = (vel_tmp >> 4);
  frame.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  frame.data[4] = kp_tmp;
  frame.data[5] = (kd_tmp >> 4);
  frame.data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  frame.data[7] = tor_tmp;
}

void DmMotorWriter::PosVelControl(can_frame& frame,
                                  const ControlCommand& control_command,
                                  bool need_position_limit) {
  // soft limit
  float position_limit = control_command.position;
  if (need_position_limit) {
    position_limit =
        std::max(motor_info_.position_min,
                 std::min(position_limit, motor_info_.position_max));
  }

  float velocity_limit = std::max(
      -motor_info_.vmax, std::min(control_command.velocity, motor_info_.vmax));

  // positon velocity id = CAN_ID + 0x100
  frame.can_id = motor_info_.motor_write_info.id + 0x100;
  frame.can_dlc = 8;

  uint8_t *pbuf, *vbuf;
  pbuf = (uint8_t*)&position_limit;
  vbuf = (uint8_t*)&velocity_limit;

  frame.data[0] = *pbuf;
  frame.data[1] = *(pbuf + 1);
  frame.data[2] = *(pbuf + 2);
  frame.data[3] = *(pbuf + 3);
  frame.data[4] = *vbuf;
  frame.data[5] = *(vbuf + 1);
  frame.data[6] = *(vbuf + 2);
  frame.data[7] = *(vbuf + 3);

  // TEST 2:
  // std::array<uint8_t, 8> data_buf{};
  //           memcpy(data_buf.data(), &position_limit, sizeof(float));
  //           memcpy(data_buf.data() + 4, &velocity_limit, sizeof(float));
  // frame.data[0] = data_buf[0];
  // frame.data[1] = data_buf[1];
  // frame.data[2] = data_buf[2];
  // frame.data[3] = data_buf[3];
  // frame.data[4] = data_buf[4];
  // frame.data[5] = data_buf[5];
  // frame.data[6] = data_buf[6];
  // frame.data[7] = data_buf[7];
}

void DmMotorWriter::Enable(can_frame& frame) {
  frame.can_id = motor_info_.motor_write_info.id;
  frame.can_dlc = 8;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;
  frame.data[7] = 0xFC;
}

void DmMotorWriter::Disable(can_frame& frame) {
  frame.can_id = motor_info_.motor_write_info.id;
  frame.can_dlc = 8;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;
  frame.data[7] = 0xFD;
}

void DmMotorWriter::SetZeroPosition(can_frame& frame) {
  frame.can_id = motor_info_.motor_write_info.id;
  frame.can_dlc = 8;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;
  frame.data[7] = 0xFE;
}

void DmMotorWriter::SwitchControlMode(const ControlMode& mode,
                                      can_frame& frame) {
  uint8_t write_data[4] = {(uint8_t)mode, 0x00, 0x00, 0x00};
  uint8_t RID = 10;
  frame = WriteMotorParam(RID, write_data);
}

void DmMotorWriter::SetPositionKp(can_frame& frame, float kp) {
  uint8_t* data_uint8;
  data_uint8 = (uint8_t*)&kp;
  uint8_t RID = 27;
  frame = WriteMotorParam(RID, data_uint8);
}

void DmMotorWriter::SetVelocityKd(can_frame& frame, float kd) {
  uint8_t* data_uint8;
  data_uint8 = (uint8_t*)&kd;
  uint8_t RID = 25;
  frame = WriteMotorParam(RID, data_uint8);
}

void DmMotorWriter::SetCurrentBW(can_frame& frame, float ibw) {
  uint8_t* data_uint8;
  data_uint8 = (uint8_t*)&ibw;
  uint8_t RID = 24;
  frame = WriteMotorParam(RID, data_uint8);
}

void DmMotorWriter::SetSpeedEnhanceFactor(can_frame& frame, float factor) {
  uint8_t* data_uint8;
  data_uint8 = (uint8_t*)&factor;
  uint8_t RID = 34;
  frame = WriteMotorParam(RID, data_uint8);
}

can_frame DmMotorWriter::ReadMotorParam(uint8_t rid) const {
  uint16_t id = motor_info_.motor_write_info.id;
  uint8_t id_low = id & 0xff;
  uint8_t id_high = (id >> 8) & 0xff;

  can_frame frame;
  frame.can_id = 0x7FF;
  frame.can_dlc = 8;

  frame.data[0] = id_low;
  frame.data[1] = id_high;
  frame.data[2] = 0x33;
  frame.data[3] = rid;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  return frame;
}

can_frame DmMotorWriter::WriteMotorParam(uint8_t rid, uint8_t data[4]) const {
  uint16_t id = motor_info_.motor_write_info.id;
  uint8_t can_low = id & 0xff;
  uint8_t can_high = (id >> 8) & 0xff;

  can_frame frame;
  frame.can_id = 0x7FF;
  frame.can_dlc = 8;

  frame.data[0] = can_low;
  frame.data[1] = can_high;
  frame.data[2] = 0x55;
  frame.data[3] = rid;
  frame.data[4] = data[0];
  frame.data[5] = data[1];
  frame.data[6] = data[2];
  frame.data[7] = data[3];

  return frame;
}

}  // namespace metal
}  // namespace makermods