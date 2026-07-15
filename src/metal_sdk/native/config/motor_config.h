#pragma once

#include <linux/can.h>

#include <string>
#include <vector>

namespace makermods {
namespace metal {

struct MotorReadInfo {
  canid_t id;
  std::string class_type;
};

struct MotorWriteInfo {
  canid_t id;
  std::string class_type;
};

struct MotorInfo {
  // correspond to (J1, J2, J3, J4, J5, J6, Gripper)
  std::string joint_name;
  MotorReadInfo motor_read_info;
  MotorWriteInfo motor_write_info;
  // 电机控制幅度
  float pmax;
  float vmax;
  float tmax;
  // min and max joint position limit（unit：degree）
  float position_min;
  float position_max;
  // mit mode control parameter
  float mit_kp;
  float mit_kd;
  // follower arm mit parameter
  float follow_mit_kp;
  float follow_mit_kd;
  // position kp of position velocity mode
  float pos_kp;
  // velocity kp of position velocity mode
  float vel_kp;
  // current bandwidth
  float ibw;
  // 速度环增强系数
  float speed_enhance_factor;
  // 极对数
  float pole_pairs;
  // 磁链
  float magnet_link;
  // 减速比
  float reduction_ratio;
  // 齿轮系数
  float gear_coefficient;
};

// 外部声明配置变量
extern const std::vector<MotorInfo> kMotorInfos;

}  // namespace metal
}  // namespace makermods