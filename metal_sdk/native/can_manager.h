#pragma once

#include <fcntl.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/motor_state.h"
#include "kinodynamic/kdl_solver.h"
#include "motor_interface_base/motor_reader_base.h"
#include "motor_interface_base/motor_writer_base.h"
#include "trajectory/polynomial_interpolation.h"
#include "trajectory/trajectory_time_allocator.h"

namespace makermods {
namespace metal {

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

class CanManager {
 public:
  CanManager() = delete;

  explicit CanManager(const std::string& can_id, const std::string& urdf_path,
                      int arm_end_type, bool enable_arm);

  ~CanManager();

  bool Init();

  double distanceToAngle(double distance) const;

  double angleToDistance(double angle) const;

  /**
   * @brief the interface of arm all joint names.
   * @return 6 or 7(include gripper) joint names.
   */
  std::vector<std::string> GetJointNames();

  /**
   * @brief the interface of all motor temperature.
   * @return 6 or 7(include gripper) joint motor temperature.
   */
  std::vector<double> GetRotorTemperature();

  /**
   * @brief the interface of all joint error code.
   * @return 6 or 7(include gripper) joint error code.
   */
  std::vector<int> GetJointErrorCode();

  /**
   * @brief the interface of all motor current.
   * @return 6 or 7(include gripper) motor current.
   */
  std::vector<double> GetMotorCurrent();

  std::vector<double> GetJointPosition();

  std::vector<double> GetJointVelocity();

  std::vector<double> GetJointEffort();

  std::array<double, 6> GetArmEndPose();

  std::vector<double> ComputeGravityTorque(
      const std::vector<double>& joint_position);

  void SetArmControlMode(int mode);

  void SetArmJointPosition(const std::array<double, 6>& arm_joint_position,
                           int velocity_ratio);

  /**
   * @brief set follow arm joint position control command.
   */
  void SetArmJointPosition(const std::vector<double>& arm_joint_position);

  void SetArmEndPose(const std::array<double, 6>& arm_end_pose);

  /**
   * @brief set gripper stroke(Unit: mm) and velocity ratio(1-10).
   * @param gripper_stroke Unit: mm
   * @param velocity_ratio 1 - 10
   */
  void SetGripperStroke(double gripper_stroke, int velocity_ratio);

  void SetEnableArm(bool enable_flag);

  void SaveJ6ZeroPosition();

 private:
  bool OpenCanDevice(const std::string& can_id);

  /**
   * @brief one thread for read motor can frame
   */
  void GenerateReaderThread();

  /**
   * @brief one thread for write can frame to control motor
   */
  void GenerateControlThread();

  bool IsCollisionDetected(std::vector<ControlCommand>& next_control_command);

  void KeepCurrentPostion(const std::vector<double>& current_position,
                          bool& is_mit_mode,
                          std::vector<ControlCommand>& next_control_command);

  bool WriteCanFrame(const can_frame& frame) const;

  bool RecvCanFrame(bool read_save_write = false);

  void MitControl(const std::shared_ptr<MotorWriterBase>& motor,
                  const ControlCommand& control_command,
                  bool need_position_limit = true);

  void PosVelControl(const std::shared_ptr<MotorWriterBase>& motor,
                     const ControlCommand& control_command,
                     bool need_position_limit = true);

  void SwitchControlMode(const std::shared_ptr<MotorWriterBase>& motor,
                         ControlMode mode);

  bool EnableArm(bool enable, bool need_read = false);

  bool CheckOfflineMotors();

 private:
  // socket can
  int socket_;

  // thread
  std::thread control_thread_;
  std::thread reader_thread_;
  std::mutex arm_control_mutux_;
  std::mutex gripper_control_mutux_;
  std::shared_mutex rw_mutex_;
  std::atomic<bool> stop_flag_ = false;

  // readers and writers
  std::vector<std::shared_ptr<MotorReaderBase>> motor_readers_;
  std::vector<std::shared_ptr<MotorWriterBase>> motor_writers_;
  std::unordered_map<canid_t, TimePoint> last_msg_time_;  // 电机ID -> 时间戳
  // const std::chrono::milliseconds short_time_debug_{50};
  const std::chrono::milliseconds offline_min_threshold_{1000};
  const std::chrono::milliseconds offline_max_threshold_{1010};

  // kdl solver
  KdlSolver kdl_solver_;

  // trajectory generator
  TrajectoryTimeAllocator trajectory_time_allocator_;
  std::unique_ptr<PolynomialInterpolation> trajectory_generator_;

  // config
  std::string can_id_;
  int arm_end_type_;
  bool enable_arm_;
  bool load_end_motor_;

  // normal control data
  std::array<double, 6> target_joint_position_;
  // should be [0.1, 1]
  double joint_speed_scale_ = 0.5;
  std::array<double, 6> last_target_joint_position_;
  std::array<double, 6> last_control_joint_position_;
  std::array<double, 6> last_control_joint_velocity_;
  std::array<double, 6> last_control_joint_acc_;

  const double gripper_gear_ratio_ = 0.029878;
  double target_gripper_joint_position_ = 0;
  // should be [0.1, 1]
  double gripper_speed_scale_ = 0.5;
  double last_control_gripper_position_;
  double last_control_gripper_velocity_;
  double last_control_gripper_acc_;

  // control 6 tof arm
  std::chrono::steady_clock::time_point last_target_position_time_;
  bool has_last_target_position_time_ = false;
  double arm_control_dt_;

  // control gripper
  std::chrono::steady_clock::time_point last_gripper_position_time_;
  bool has_last_gripper_position_time_ = false;
  double gripper_control_dt_;

  // follow arm data
  std::vector<double> follow_target_joint_position_;
  bool synchronous_master_arm_ = false;
  bool need_synchronous_ = true;

  std::string mode_ = "NRT_JOINT_POSITION";

  std::vector<std::vector<ControlCommand>> arm_control_trajectory_;
  std::vector<ControlCommand> gripper_control_trajectory_;
  int arm_control_index_ = 0;
  int gripper_control_index_ = 0;
  int max_size_ = 0;
  bool new_arm_target_position_ = false;
  bool new_gripper_target_position_ = false;
  std::map<int, int> per_motor_overload_frame_counter_;
  bool init_joint_soft_limit_ = false;
  bool init_trajectory_planning_ = false;

  std::vector<double> distances_ = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,  16,
      17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,  33,
      34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,  50,
      51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66,  67,
      68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83,  84,
      85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 102.5};
  std::vector<double> angles_ = {
      0.002,    0.01407,  0.0368934, 0.08634,  0.115854, 0.161084, 0.176609,
      0.194816, 0.22893,  0.262277,  0.295,    0.314215, 0.344305, 0.362704,
      0.393177, 0.411575, 0.441473,  0.46773,  0.482871, 0.508743, 0.527718,
      0.550333, 0.576397, 0.60227,   0.621053, 0.639452, 0.662833, 0.683915,
      0.707105, 0.722054, 0.748693,  0.763451, 0.782233, 0.805039, 0.815964,
      0.841837, 0.856786, 0.879593,  0.894158, 0.91639,  0.92789,  0.94648,
      0.969287, 0.987494, 0.998418,  1.02084,  1.03617,  1.05112,  1.0701,
      1.08505,  1.10383,  1.11897,   1.13411,  1.15557,  1.16382,  1.18221,
      1.20117,  1.21594,  1.23779,   1.24986,  1.26424,  1.28264,  1.29452,
      1.31273,  1.3317,   1.34359,   1.36505,  1.38019,  1.39169,  1.40683,
      1.42542,  1.44401,  1.45609,   1.47468,  1.48943,  1.50419,  1.52259,
      1.53735,  1.55575,  1.57453,   1.582,    1.60462,  1.62302,  1.64218,
      1.65694,  1.66825,  1.68722,   1.70984,  1.729,    1.73973,  1.76216,
      1.78477,  1.79991,  1.82214,   1.84399,  1.86297,  1.88577,  1.9036,
      1.92717,  1.94844,  1.97527,   2.03143};
};

}  // namespace metal
}  // namespace makermods
