#include "config/motor_config.h"
#include <cmath>

namespace makermods {
namespace metal {

const std::vector<MotorInfo> kMotorInfos = {
    {                          // MotorInfo 第一个电机
     "J1",                     // joint_name
     {0x11, "DmMotorReader"},  // motor_read_info
     {0x01, "DmMotorWriter"},  // motor_write_info
     6.28,                     // pmax
     10,                       // vmax
     30,                       // tmax
     -160 * M_PI / 180,                     // position_min
     160 * M_PI / 180,                      // position_max
     300,                      // mit_kp
     5,                        // mit_kd
     200,                       // follow_mit_kp
     3,                        // follow_mit_kd
     100,                       // pos_kp
     0.00384,                  // vel_kp
     1000,                     // ibw
     500,
     14,
     0.005055572,
     40,
     1},
    {                          // MotorInfo 第二个电机
     "J2",                     // joint_name
     {0x12, "DmMotorReader"},  // motor_read_info
     {0x02, "DmMotorWriter"},  // motor_write_info
     6.28,                    // pmax
     10,                       // vmax
     120,                       // tmax
     -180 * M_PI / 180,                     // position_min
     0,                        // position_max
     500,                      // mit_kp
     5,                        // mit_kd
     500,                       // follow_mit_kp
     5,                        // follow_mit_kd
     1500,                       // pos_kp
     0.005,                 // vel_kp
     1000,                     // ibw
     500,
     14,
     0.003540293,
     48,
     1},
    {                          // MotorInfo 第三个电机
     "J3",                     // joint_name
     {0x13, "DmMotorReader"},  // motor_read_info
     {0x03, "DmMotorWriter"},  // motor_write_info
     6.28,                    // pmax
     10,                       // vmax
     30,                       // tmax
     0,                        // position_min
     180 * M_PI / 180,                      // position_max
     500,                      // mit_kp
     5,                        // mit_kd
     400,                       // follow_mit_kp
     5,                        // follow_mit_kd
     3000,                       // pos_kp
     0.006,             // vel_kp
     1000,                     // ibw
     500,
     14,
     0.005062674,
     40,
     1},
    {                          // MotorInfo 第四个电机
     "J4",                     // joint_name
     {0x14, "DmMotorReader"},  // motor_read_info
     {0x04, "DmMotorWriter"},  // motor_write_info
     6.28,                     // pmax
     10,                       // vmax
     30,                       // tmax
     -123 * M_PI / 180,                      // position_min
     81 * M_PI / 180,                       // position_max
     400,                       // mit_kp
     4.0,                        // mit_kd
     200,                       // follow_mit_kp
     2,                        // follow_mit_kd
     300,                       // pos_kp
     0.00384,                  // vel_kp
     500,                     // ibw
     500,
     14,
     0.005062674,
     10,
     1},
    {                          // MotorInfo 第五个电机
     "J5",                     // joint_name
     {0x15, "DmMotorReader"},  // motor_read_info
     {0x05, "DmMotorWriter"},  // motor_write_info
     6.28,                     // pmax
     30,                       // vmax
     20,                       // tmax
     -85 * M_PI / 180,                      // position_min
     85 * M_PI / 180,                       // position_max
     25,                       // mit_kp
     0.2,                        // mit_kd
     20,                       // follow_mit_kp
     0.1,                        // follow_mit_kd
     200,                       // pos_kp
     0.00384,                  // vel_kp
     1000,                     // ibw
     500,
     14,
     0.00494409,
     10,
     1},
    {                          // MotorInfo 第六个电机
     "J6",                     // joint_name
     {0x16, "DmMotorReader"},  // motor_read_info
     {0x06, "DmMotorWriter"},  // motor_write_info
     6.28,                     // pmax
     30,                       // vmax
     20,                       // tmax
     -145 * M_PI / 180,                      // position_min
     145 * M_PI / 180,                       // position_max
     20,                       // mit_kp
     0.2,                        // mit_kd
     20,                       // follow_mit_kp
     0.1,                        // follow_mit_kd
     300,                       // pos_kp
     0.00384,                  // vel_kp
     1000,                     // ibw
     500,
     14,
     0.005202444,
     10,
     1},
    {                          // MotorInfo 末端夹爪或示教器
     "Gripper",                // joint_name
     {0x17, "DmMotorReader"},  // motor_read_info
     {0x07, "DmMotorWriter"},  // motor_write_info
     6.28,                     // pmax
     30,                       // vmax
     20,                       // tmax
     0,                      // position_min
     2.4,                       // position_max
     20,                       // mit_kp
     0.1,                        // mit_kd
     20,                       // follow_mit_kp
     0.1,                        // follow_mit_kd
     100,                       // pos_kp
     0.00384,                  // vel_kp
     1000,                     // ibw
     500,
     14,
     0.005249193,
     10,
     1}};

}  // namespace metal
}  // namespace makermods
