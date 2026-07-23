#pragma once

#include <vector>
#include <string>

namespace makermods {
namespace metal {

enum Status {
  Disabled = 0,              // 失能
  Enabled = 1,                // 使能 
  MotorDisconnected = 2,      // 电机断开
  OverVoltage = 3,            // 电压过大
  UnderVoltage = 4,           // 电压过低
  Overcurrent = 5,            // 过电流
  MosOverTemperature = 6,        // MOS 过温
  RotorOverTemperature = 7,    // 电机线圈过温
  Overload = 8,                // 过载
};

struct MotorState {
  std::string name;
  Status status;             // error code 
  float position;      
  float velocity;
  float torque;
  float current;            // 电流
  float rotor_temperature;  // 线圈温度
  float mos_temperature;    // MOS温度
};

using MotorStateVector = std::vector<MotorState>;

struct ControlCommand {
  float position;
  float velocity;
  float acc;
  float kp;
  float kd;
  float torque;
};

enum ControlMode { MIT = 1, POS_VEL = 2, VEL = 3 };

}  // namespace metal
}  // namespace makermods