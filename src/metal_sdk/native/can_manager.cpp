#include "can_manager.h"

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "common/log.h"
#include "common/motor_state.h"
#include "common/time.h"
#include "config/motor_config.h"
#include "motor_interface_base/motor_writer_base.h"
#include "motor_readers/dm_motor_reader.h"
#include "motor_writers/dm_motor_writer.h"
#include "trajectory/polynomial_interpolation.h"

namespace makermods {
namespace metal {

CanManager::CanManager(const std::string& can_id, const std::string& urdf_path,
                       int arm_end_type, bool enable_arm)
    : can_id_(can_id),
      arm_end_type_(arm_end_type),
      kdl_solver_(urdf_path, arm_end_type),
      enable_arm_(enable_arm) {
  trajectory_generator_ = std::make_unique<PolynomialInterpolation>();
}

CanManager::~CanManager() {
  stop_flag_.store(true);

  // for safety, not disable motor
  // wait for thread finish
  if (control_thread_.joinable()) {
    AINFO << std::this_thread::get_id() << " control_thread join";
    control_thread_.join();
  }

  // wait for thread finish
  if (reader_thread_.joinable()) {
    AINFO << std::this_thread::get_id() << " reader_thread join";
    reader_thread_.join();
  }

  close(socket_);
}

/**
 * @brief bring up the kdl solver, CAN socket and per-motor reader/writer
 * objects, enable the arm and start the reader/control threads.
 */
bool CanManager::Init() {
  // init kdl solver
  if (!kdl_solver_.Init()) {
    AERROR << "Init kdl solver failed.";
    return false;
  }

  // open can device
  if (!OpenCanDevice(can_id_)) {
    AERROR << "Failed to open can device:" << can_id_;
    return false;
  }

  // if load end motor
  if (arm_end_type_ == 0) {
    load_end_motor_ = false;
  } else if (arm_end_type_ == 1 || arm_end_type_ == 2 || arm_end_type_ == 3) {
    load_end_motor_ = true;
  } else {
    AERROR << "arm_end_type is error: " << arm_end_type_;
    return false;
  }

  if (kMotorInfos.size() != 7) {
    AERROR << "motor config size != 7";
    return false;
  }

  int motor_size =
      load_end_motor_ ? kMotorInfos.size() : kMotorInfos.size() - 1;
  for (int i = 0; i < motor_size; i++) {
    // reader
    auto motor_info = kMotorInfos.at(i);
    if (motor_info.motor_read_info.class_type == "DmMotorReader") {
      std::shared_ptr<MotorReaderBase> reader =
          std::make_shared<DmMotorReader>();
      reader->Init(motor_info);

      motor_readers_.emplace_back(reader);
    } else {
      AERROR << "Failed create " << motor_info.motor_read_info.class_type;
      return false;
    }

    // writer
    if (motor_info.motor_write_info.class_type == "DmMotorWriter") {
      std::shared_ptr<MotorWriterBase> writer =
          std::make_unique<DmMotorWriter>();
      writer->Init(motor_info);

      motor_writers_.emplace_back(writer);
    } else {
      AERROR << "Failed create " << motor_info.motor_write_info.class_type;
      return false;
    }
  }

  // motor enable or disable
  if (!EnableArm(enable_arm_, true)) {
    return false;
  }
  // generate one thread for can read data (must be before generate writer
  // thread)
  reader_thread_ = std::thread(&CanManager::GenerateReaderThread, this);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // The initial position at the time of arm power-on
  follow_target_joint_position_.resize(motor_size, 0);

  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < motor_size; i++) {
      float motor_position = motor_readers_.at(i)->motor_state().position;
      // joint soft limit
      if (motor_position > kMotorInfos.at(i).position_max) {
        motor_position = kMotorInfos.at(i).position_max;
        init_joint_soft_limit_ = true;
        init_trajectory_planning_ = true;
      }

      if (motor_position < kMotorInfos.at(i).position_min) {
        motor_position = kMotorInfos.at(i).position_min;
        init_joint_soft_limit_ = true;
        init_trajectory_planning_ = true;
      }

      // normal control arm
      if (i < 6) {
        target_joint_position_.at(i) = motor_position;
      } else {
        target_gripper_joint_position_ = motor_position;
      }

      // slave arm
      follow_target_joint_position_.at(i) = motor_position;
    }
  }

  // generate one thread for control motor
  control_thread_ = std::thread(&CanManager::GenerateControlThread, this);

  if (init_joint_soft_limit_ &&
      (mode_ == "slave_arm" || mode_ == "NRT_JOINT_POSITION")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return true;
}

/**
 * @brief create, bind and configure the raw CAN socket for can_id.
 */
bool CanManager::OpenCanDevice(const std::string& can_id) {
  // create socket
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    AERROR << "Error while opening CAN socket";
    return false;
  }

  // specifying the can device id
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_id.c_str(), IFNAMSIZ);
  if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
    AERROR << "Failed to ioctrl";
    return false;
  }

  // 设置为非阻塞模式
  // int flags = fcntl(socket_, F_GETFL, 0);
  // fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

  // bind socket to can device
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    AERROR << "Error in CAN socket bind";
    return false;
  }

  // set read timeout 1s
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  return true;
}

/**
 * @brief map gripper stroke distance (mm) to motor angle via the
 * calibrated distances_/angles_ lookup table (nearest-neighbor).
 */
double CanManager::distanceToAngle(double distance) const {
  // 边界判断
  if (distances_.empty()) return 0.0;
  if (distance <= distances_.front()) return angles_.front();
  if (distance >= distances_.back()) return angles_.back();

  // 直接查找最近行程（唯一对应）
  for (size_t i = 1; i < distances_.size(); ++i) {
    if (distance < distances_[i]) {
      double prev = distances_[i - 1];
      double next = distances_[i];
      if (std::fabs(distance - prev) < std::fabs(next - distance))
        return angles_[i - 1];
      else
        return angles_[i];
    }
  }
  return angles_.back();
}

/**
 * @brief inverse of distanceToAngle: map motor angle to gripper stroke
 * distance (mm) via the calibrated lookup table.
 */
double CanManager::angleToDistance(double angle) const {
  // 边界判断
  if (angles_.empty()) return 0.0;
  if (angle <= angles_.front()) return distances_.front();
  if (angle >= angles_.back()) return distances_.back();

  // 查找最近角度
  for (size_t i = 1; i < angles_.size(); ++i) {
    if (angle < angles_[i]) {
      double prev = angles_[i - 1];
      double next = angles_[i];
      // 比较更接近哪一个角度
      if (std::fabs(angle - prev) < std::fabs(next - angle))
        return distances_[i - 1];
      else
        return distances_[i];
    }
  }
  return distances_.back();
}

std::vector<std::string> CanManager::GetJointNames() {
  std::vector<std::string> joint_names;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < motor_readers_.size(); i++) {
      joint_names.emplace_back(motor_readers_.at(i)->name());
    }
  }

  return joint_names;
}

std::vector<double> CanManager::GetRotorTemperature() {
  std::vector<double> rotor_temperature;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < motor_readers_.size(); i++) {
      rotor_temperature.emplace_back(
          motor_readers_.at(i)->motor_state().rotor_temperature);
    }
  }

  return rotor_temperature;
}

std::vector<int> CanManager::GetJointErrorCode() {
  std::vector<int> error_code;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < motor_readers_.size(); i++) {
      error_code.emplace_back(
          static_cast<int>(motor_readers_.at(i)->motor_state().status));
    }
  }

  return error_code;
}

std::vector<double> CanManager::GetMotorCurrent() {
  std::vector<double> motor_current;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < motor_readers_.size(); i++) {
      motor_current.emplace_back(motor_readers_.at(i)->motor_state().current);
    }
  }

  return motor_current;
}

/**
 * @brief current joint positions (rad); gripper entry (if present) is
 * converted from motor angle to stroke distance via angleToDistance.
 */
std::vector<double> CanManager::GetJointPosition() {
  std::vector<double> joint_position;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < motor_readers_.size(); i++) {
      if (i < 6) {
        joint_position.emplace_back(
            motor_readers_.at(i)->motor_state().position);
      } else {
        // AINFO_EVERY(1) << "gripper_motor position: "
        //                << motor_readers_.at(i)->motor_state().position;
        // double gripper = std::fabs(
        //     motor_readers_.at(i)->motor_state().position / gripper_gear_ratio_);
        double gripper = angleToDistance(motor_readers_.at(i)->motor_state().position);
        joint_position.emplace_back(gripper);
        // AINFO << "gripper joint position: " <<
        // motor_readers_.at(i)->motor_state().position;
      }
    }
  }

  return joint_position;
}

/**
 * @brief current joint velocities as reported by the motor readers.
 */
std::vector<double> CanManager::GetJointVelocity() {
  std::vector<double> joint_velocity;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < motor_readers_.size(); i++) {
      joint_velocity.emplace_back(motor_readers_.at(i)->motor_state().velocity);
    }
  }

  return joint_velocity;
}

/**
 * @brief current joint torque/effort as reported by the motor readers.
 */
std::vector<double> CanManager::GetJointEffort() {
  std::vector<double> joint_effort;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < motor_readers_.size(); i++) {
      joint_effort.emplace_back(motor_readers_.at(i)->motor_state().torque);
    }
  }

  return joint_effort;
}

/**
 * @brief current end-effector pose (xyz+rpy), computed from the current
 * joint positions via forward kinematics.
 */
std::array<double, 6> CanManager::GetArmEndPose() {
  std::array<double, 6> arm_joint_position;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < 6; i++) {
      arm_joint_position.at(i) = (motor_readers_.at(i)->motor_state().position);
    }
  }

  std::array<double, 6> end_pose;
  // arm kinematics compute end pose
  kdl_solver_.FkSolver(arm_joint_position, end_pose);

  return end_pose;
}

/**
 * @brief select the high-level arm mode: 0=gravity compensation (drag
 * teaching), 1=real-time joint position follow, 2=non-real-time joint
 * position with trajectory planning.
 */
void CanManager::SetArmControlMode(int mode) {
  // ControlMode
  if (mode == 0) {
    // GRAVITY_COMPENSATION: master arm drag teaching
    mode_ = "master_arm";
    AINFO << "set master arm.";
  } else if (mode == 1) {
    // RT_JOINT_POSITION: slave arm fast follow
    mode_ = "slave_arm";
    AINFO << "set slave arm.";
  } else if (mode == 2) {
    // NRT_JOINT_POSITION: normal control arm need to trajectory planning
    mode_ = "NRT_JOINT_POSITION";
    AINFO << "set non-real-time joint position control.";
  } else {
    // NOTE(known-limitation): other control mode 未覆盖。见 docs/metal_sdk_known_limitations.md
    AWARN << "Invalid control mode : " << mode;
  }
}

/**
 * @brief set target joint positions (clamped to soft limits) and follow
 * speed ratio for the normal (non-follow) arm control path.
 */
void CanManager::SetArmJointPosition(
    const std::array<double, 6>& arm_joint_position, int velocity_ratio) {
  {
    std::lock_guard<std::mutex> lock(arm_control_mutux_);
    // joint position
    for (int i = 0; i < 6; ++i) {
      // joint soft limit
      double position = std::fmax(
          kMotorInfos.at(i).position_min,
          std::fmin(arm_joint_position.at(i), kMotorInfos.at(i).position_max));
      target_joint_position_.at(i) = position;
    }

    // 监听控制频率（上一次和这一次下发的时间间隔）
    auto current_time = std::chrono::steady_clock::now();
    if (has_last_target_position_time_) {
      double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                      current_time - last_target_position_time_)
                      .count();
      if (dt > 0.19) {
        // 低于5hz, 按照正常到点规划
        has_last_target_position_time_ = false;
        AINFO << "dt > 0.19";
      } else {
        // 高于5hz，末端规划速度不能为0
        arm_control_dt_ = dt;
      }
      // AINFO << "[SetArmJointPosition] Time since last call: " << dt << " s";
    } else {
      // AINFO << "[SetArmJointPosition] First call";
    }
    last_target_position_time_ = current_time;

    // joint velocity
    int limit_vel = std::max(1, std::min(10, velocity_ratio));
    joint_speed_scale_ = limit_vel / 10.0;

    new_arm_target_position_ = true;
  }
}

/**
 * @brief set follow arm joint position control command.
 */
void CanManager::SetArmJointPosition(
    const std::vector<double>& arm_joint_position) {
  // check joint position size
  int size = arm_joint_position.size();
  if (load_end_motor_) {
    if (size != 7) {
      AFATAL << "joint position size must be 7";
      return;
    }
  } else {
    if (size != 6) {
      AFATAL << "joint position size must be 6";
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(arm_control_mutux_);
    for (int i = 0; i < arm_joint_position.size(); ++i) {
      // joint soft limit
      if (i < 6) {
        double position = std::fmax(kMotorInfos.at(i).position_min,
                                    std::fmin(arm_joint_position.at(i),
                                              kMotorInfos.at(i).position_max));
        follow_target_joint_position_.at(i) = position;
      } else {
        double gripper_distance =
            std::fmax(0, std::fmin(arm_joint_position.at(i), 100));    
        // follow_target_joint_position_.at(i) =
        //     std::fabs(gripper_distance * gripper_gear_ratio_);
        follow_target_joint_position_.at(i) =
            std::fabs(distanceToAngle(gripper_distance));

        // AINFO << "gripper_joint_position: "
        //       << follow_target_joint_position_.at(i)
        //       << " ,gripper_distance: " << gripper_distance;
      }
    }

    if (need_synchronous_) {
      need_synchronous_ = false;
      synchronous_master_arm_ = true;
    }
  }
}

/**
 * @brief set target end-effector pose; solves IK to joint positions and
 * forwards them to both the normal and follow control targets.
 */
void CanManager::SetArmEndPose(const std::array<double, 6>& arm_end_pose) {
  // arm Inverse Kinematics compute joint position.
  auto start_time = std::chrono::high_resolution_clock::now();
  std::array<double, 6> ik_joint_position;
  std::array<double, 6> current_joint_position;
  {
    std::shared_lock lock(rw_mutex_);  // 获取读锁
    for (int i = 0; i < 6; i++) {
      current_joint_position.at(i) =
          motor_readers_.at(i)->motor_state().position;
    }
  }

  if (!kdl_solver_.IkSolverWithTracIK(arm_end_pose, current_joint_position,
                                      ik_joint_position)) {
    AERROR << "Inverse Kinematics failed.";

  } else {
    {
      std::lock_guard<std::mutex> lock(arm_control_mutux_);
      // for normal control arm
      for (int i = 0; i < 6; ++i) {
        // joint soft limit
        double position = std::fmax(
            kMotorInfos.at(i).position_min,
            std::fmin(ik_joint_position.at(i), kMotorInfos.at(i).position_max));
        target_joint_position_.at(i) = position;
      }

      // for follow arm
      for (int i = 0; i < 6; i++) {
        follow_target_joint_position_.at(i) = ik_joint_position.at(i);
      }

      // 监听控制频率（上一次和这一次下发的时间间隔）
      auto current_time = std::chrono::steady_clock::now();
      if (has_last_target_position_time_) {
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                        current_time - last_target_position_time_)
                        .count();
        if (dt > 0.19) {
          // 低于5hz, 按照正常到点规划
          has_last_target_position_time_ = false;
        } else {
          // 高于5hz，末端规划速度不能为0
          arm_control_dt_ = dt;
        }
        // AINFO << "[SetArmEndPose] Time since last call: " << dt << " s";
      }
      last_target_position_time_ = current_time;
      new_arm_target_position_ = true;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  // AINFO << "Inverse Kinematics time:"
  //       << std::chrono::duration<double, std::milli>(end_time - start_time)
  //              .count()
  //       << " ms";
}

void CanManager::SetGripperStroke(double gripper_stroke, int velocity_ratio) {
  double gripper_distance = std::fmax(0, std::fmin(gripper_stroke, 100));
  {
    std::lock_guard<std::mutex> lock(gripper_control_mutux_);
    if (load_end_motor_) {
      // target_gripper_joint_position_ =
      //     std::fabs(gripper_distance * gripper_gear_ratio_);
      target_gripper_joint_position_ =
          std::fabs(distanceToAngle(gripper_distance));

      auto current_time = std::chrono::steady_clock::now();
      if (has_last_gripper_position_time_) {
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                        current_time - last_gripper_position_time_)
                        .count();
        if (dt > 0.19) {
          // 低于5hz, 按照正常到点规划
          has_last_gripper_position_time_ = false;
        } else {
          // 高于5hz，末端规划速度不能为0
          gripper_control_dt_ = dt;
        }
        // AINFO << "[SetGripperStroke] Time since last call: " << dt << " s";
      }

      last_gripper_position_time_ = current_time;

      // joint velocity
      int limit_vel = std::max(1, std::min(10, velocity_ratio));
      gripper_speed_scale_ = limit_vel / 10.0;
    }

    new_gripper_target_position_ = true;
  }
}

/**
 * @brief enable/disable all arm motors.
 */
void CanManager::SetEnableArm(bool enable_flag) { EnableArm(enable_flag); }

/**
 * @brief persist the current J6 (wrist) position as the motor's zero
 * point; only allowed while the motor is disabled, for safety.
 */
void CanManager::SaveJ6ZeroPosition() {
  // 为了安全，必须要处于失能状态下，再保存位置零点
  if (!motor_readers_.at(5)->IsDisEnable()) {
    AERROR << "The motor is not disable. For safety, can't save zero position.";
    return;
  }

  can_frame save_frame;
  motor_writers_.at(5)->SetZeroPosition(save_frame);
  if (!WriteCanFrame(save_frame)) {
    AERROR << "Failed to save J6 zero position.";
  } else {
    AINFO << "Success save J6 zero position.";
  }
}

/**
 * @brief blocking read of one CAN frame and dispatch it to the matching
 * motor reader by CAN id.
 */
bool CanManager::RecvCanFrame(bool read_save_write) {
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  int nbytes = read(socket_, &frame, sizeof(struct can_frame));
  if (nbytes < 0) {
    AERROR << "CAN read error, Check the wiring between the motors.";
    return false;
  } else {
    // parse can frame
    for (int i = 0; i < motor_readers_.size(); i++) {
      auto motor = motor_readers_.at(i);
      if (motor->id() == frame.can_id) {
        if (read_save_write) {
          motor->SetRWS();
        }

        if (!motor->ReadCanFrame(frame)) {
          AERROR << "Failed to parse can frame";
          return false;
        }

        break;
      }
    }
  }

  return true;
}

/**
 * @brief encode and send one MIT-mode (torque+pos+vel) control frame.
 */
void CanManager::MitControl(const std::shared_ptr<MotorWriterBase>& motor,
                            const ControlCommand& control_command,
                            bool need_position_limit) {
  // write
  can_frame write_frame;
  motor->MitControl(write_frame, control_command, need_position_limit);
  if (!WriteCanFrame(write_frame)) {
    AERROR << "Failed to write can frame to device: " << motor->name();
  }
}

/**
 * @brief encode and send one position/velocity-mode control frame.
 */
void CanManager::PosVelControl(const std::shared_ptr<MotorWriterBase>& motor,
                               const ControlCommand& control_command,
                               bool need_position_limit) {
  // write
  can_frame write_frame;
  motor->PosVelControl(write_frame, control_command, need_position_limit);
  if (!WriteCanFrame(write_frame)) {
    AERROR << "Failed to write can frame to device: " << motor->name();
  }
}

/**
 * @brief switch a single motor's low-level control mode (MIT/POS_VEL/VEL)
 * and read back the ack frame.
 */
void CanManager::SwitchControlMode(
    const std::shared_ptr<MotorWriterBase>& motor, ControlMode mode) {
  // write
  can_frame frame;
  motor->SwitchControlMode(mode, frame);
  if (!WriteCanFrame(frame)) {
    AERROR << "Failed send frame to change control mode " << motor->name();
  } else {
    if (mode == ControlMode::MIT) {
      AINFO << motor->name() << " switch to Mit Control Mode.";
    } else if (mode == ControlMode::POS_VEL) {
      AINFO << motor->name() << " switch to PosVel Control Mode.";
    } else {
      AINFO << motor->name() << " switch to Vel Control Mode.";
    }
  }

  // read
  RecvCanFrame(true);

  std::this_thread::sleep_for(std::chrono::microseconds(200));
}

/**
 * @brief enable or disable every arm motor in sequence; optionally waits
 * for a CAN ack per motor to confirm the link is alive.
 */
bool CanManager::EnableArm(bool enable, bool need_read) {
  for (int i = 0; i < motor_writers_.size(); i++) {
    can_frame frame;
    auto motor = motor_writers_.at(i);
    if (enable) {
      // enable
      motor->Enable(frame);
    } else {
      // disable
      motor->Disable(frame);
    }
    WriteCanFrame(frame);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    if (need_read) {
      if (!RecvCanFrame()) {
        AERROR << "Failed connect to " << motor->name();
        return false;
      }
    }

    if (enable) {
      AINFO << "Success enable motor " << motor->name();
    } else {
      AINFO << "Success disable motor " << motor->name();
    }
  }

  return true;
}

/**
 * @brief write one CAN frame to the socket; logs the errno-specific
 * failure reason (EINTR/EAGAIN/ENETDOWN/etc.) on short/failed writes.
 */
bool CanManager::WriteCanFrame(const can_frame& frame) const {
  int bytes = write(socket_, &frame, sizeof(struct can_frame));
  if (bytes != sizeof(frame)) {
    // 获取错误码
    int error_code = errno;
    // 根据不同的错误码处理不同的情况
    switch (error_code) {
      case EINTR:
        // 写入操作被信号中断
        // 可以考虑重新尝试写入
        AINFO << "EINTR";
        break;

      case EAGAIN:  // 或 EWOULDBLOCK (通常相同值)
        // 非阻塞模式下资源暂时不可用
        // 可以稍后重试或等待可写事件
        AINFO << "EAGAIN";
        break;

      case ENETDOWN:
        // 网络接口已关闭
        // 需要重新初始化CAN接口
        AINFO << "ENETDOWN";
        break;

      case ENOBUFS:
        // 系统缓冲区不足
        // 可能需要减少发送频率或增加缓冲区
        AINFO << "ENOBUFS";
        break;

      case ENXIO:
        // CAN设备可能不存在或未正确配置
        // 检查CAN接口配置
        AINFO << "ENXIO";
        break;

      case EINVAL:
        // 无效参数，可能是socket或frame有问题
        // 检查socket和frame结构
        AINFO << "EINVAL";
        break;

      default:
        // 其他未处理的错误
        AINFO << "default";
        break;
    }

    AERROR << "Failed to write can frame: " << strerror(errno);

    return false;
  }

  return true;
}

/**
 * @brief heuristic collision proxy: flags large position tracking error
 * or sustained per-motor/simultaneous torque overload.
 */
// NOTE(known-limitation): this function and KeepCurrentPostion() below are
// implemented but not currently invoked from the control dispatch path
// (no call site pre-write); see docs/metal_sdk_known_limitations.md.
bool CanManager::IsCollisionDetected(
    std::vector<ControlCommand>& next_control_command) {
  const double kPositionThreshold = 0.05;
  // NOTE(known-limitation): 速度条件暂时添加，缺 max vel/acc/jerk 限制。见 docs/metal_sdk_known_limitations.md
  const double kVelocityZeroThreshold = 0.01;
  bool is_position_deviation = false;
  bool is_torque_overload = false;
  int frame_overload_motor_count = 0;

  for (size_t i = 0; i < next_control_command.size(); ++i) {
    const double pos_diff =
        std::abs(next_control_command[i].position - GetJointPosition()[i]);
    const double effort = std::abs(GetJointEffort()[i]);
    const double torque_limit = kMotorInfos.at(i).tmax;

    // 条件1：检测位置偏差
    if (pos_diff > kPositionThreshold) {
      is_position_deviation = true;
    }

    // 条件2：检测力矩超载
    if (effort > torque_limit) {
      ++frame_overload_motor_count;
      per_motor_overload_frame_counter_[i]++;

      // a.连续帧内单个电机超载达到3次
      if (per_motor_overload_frame_counter_[i] >= 3) {
        AWARN << "Motor [" << kMotorInfos.at(i).joint_name
              << "] reached torque limit 3 times";
        is_torque_overload = true;
      }

    } else {
      // 当前帧未超载，计数清零
      per_motor_overload_frame_counter_[i] = 0;
    }
  }

  // b.同一帧内超载电机数 ≥ 3：立即判定碰撞
  if (frame_overload_motor_count >= 3) {
    AWARN << "Torque overload detected on " << frame_overload_motor_count
          << " motors in the same frame";
    is_torque_overload = true;
  }

  return is_position_deviation || is_torque_overload;
}

/**
 * @brief build a stop command list that holds the current joint position
 * (zero velocity/torque in MIT mode, or a fixed hold velocity otherwise).
 */
void CanManager::KeepCurrentPostion(
    const std::vector<double>& current_position, bool& is_mit_mode,
    std::vector<ControlCommand>& stop_control_command) {
  stop_control_command.clear();
  ControlCommand control_command;
  // 发生碰撞时，保持当前位置
  if (is_mit_mode) {  // mit模式
    for (int i = 0; i < current_position.size(); i++) {
      // pos
      control_command.position = current_position.at(i);
      // vel
      control_command.velocity = 0;
      // kp
      control_command.kp = motor_writers_.at(i)->MitKp();
      // kd
      control_command.kd = motor_writers_.at(i)->MitKd();
      // torque
      control_command.torque = 0;
      stop_control_command.push_back(control_command);
    }
  } else {  // 位置速度模式
    for (int i = 0; i < current_position.size(); i++) {
      // pos
      control_command.position = current_position.at(i);
      // vel NOTE(known-limitation): 速度待定。见 docs/metal_sdk_known_limitations.md
      if (i < 3) {
        control_command.velocity = 5.0;
      } else {
        control_command.velocity = 20.0;
      }
      stop_control_command.push_back(control_command);
    }
  }
}

/**
 * @brief scan last-seen timestamps per motor id and report whether any
 * motor has gone silent past offline_max_threshold_.
 */
bool CanManager::CheckOfflineMotors() {
  auto now = Clock::now();
  bool result = false;
  for (const auto& [id, last_time] : last_msg_time_) {
    auto time = now - last_time;
    char buf[11];
    std::snprintf(buf, sizeof(buf), "0x%02X", id);

    if (time > offline_min_threshold_ && time < offline_max_threshold_) {
      AERROR << "Motor id [" << buf << "] offline 1s.";
    }

    if (time > offline_max_threshold_) {
      result = true;
    }
  }

  return result;
}

void CanManager::GenerateReaderThread() {
  while (!stop_flag_.load()) {
    // read can frame
    struct can_frame frame;
    int nbytes = read(socket_, &frame, sizeof(struct can_frame));

    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        AERROR << "Timeout waiting for CAN frame";
      } else {
        AERROR << "CAN read error";
      }
      stop_flag_.store(true);
      break;
    } else if (nbytes == 0) {
      AERROR << "CAN socket closed";
      stop_flag_.store(true);
      break;
    } else if (nbytes < sizeof(struct can_frame)) {
      AERROR << "CAN read incomplete";
      // stop_flag_.store(true);
      // break;
    } else {
      {
        // parse can frame
        std::unique_lock lock(rw_mutex_);  // 获取写锁
        for (int i = 0; i < motor_readers_.size(); i++) {
          auto motor = motor_readers_.at(i);
          if (motor->id() == frame.can_id) {
            if (!motor->ReadCanFrame(frame)) {
              AERROR << "Failed to parse can frame";
            }

            last_msg_time_[motor->id()] = Clock::now();

            break;
          }
        }
      }
    }

    // 检查是否有电机掉线
    if (CheckOfflineMotors()) {
      AERROR << "Stop Program!!!";
      stop_flag_.store(true);
      break;
    }
  }

  AINFO << "Can reader thread finish";
}

void CanManager::GenerateControlThread() {
  int control_hz = 400;
  LoopRate rate(control_hz);

  while (!stop_flag_.load()) {
    // auto start_time = std::chrono::high_resolution_clock::now();
    // get current motor states
    std::vector<double> current_joint_position;
    std::vector<double> current_joint_velocity;
    std::vector<double> current_joint_torque;
    {
      std::shared_lock lock(rw_mutex_);  // 获取读锁
      for (int i = 0; i < motor_readers_.size(); ++i) {
        auto motor_state = motor_readers_.at(i)->motor_state();
        current_joint_position.push_back(motor_state.position);
        current_joint_velocity.push_back(motor_state.velocity);
        current_joint_torque.push_back(motor_state.torque);
      }
    }

    // 电机速度会有波动
    for (int i = 0; i < current_joint_velocity.size(); i++) {
      if (std::fabs(current_joint_velocity.at(i)) < 0.05) {
        current_joint_velocity.at(i) = 0;
      }
    }

    // 前馈力矩
    std::vector<double> feed_forward_torque;
    kdl_solver_.FeedforwardTorqueCompensation(
        current_joint_position, current_joint_velocity, feed_forward_torque);
    // 负载识别补偿力矩
    // std::vector<double> payload_torque;

    if (mode_ == "master_arm") {
      // master arm (drag teaching)
      // 补偿末端负载带来的力矩
      // kdl_solver_.ComputePayloadCompensation(
      //     current_joint_position, current_joint_torque, payload_torque);

      for (int i = 0; i < motor_writers_.size(); i++) {
        // MIT模式下的力矩控制，除了力矩给控制值，其余参数均设为0，夹爪单独进行位置控制
        ControlCommand control_command;
        // position
        control_command.position = 0;
        // velocity
        control_command.velocity = 0;
        // kp
        control_command.kp = 0;
        // kd
        control_command.kd = 0;
        // torque
        if (i < 6) {
          control_command.torque = feed_forward_torque[i];
          // 更新目标位置
          target_joint_position_.at(i) = current_joint_position.at(i);
        } else {
          control_command.torque = kdl_solver_.GripperTorqueCompensation(
              current_joint_velocity.at(i), arm_end_type_);
          // 更新目标位置
          target_gripper_joint_position_ = current_joint_position.at(i);
        }

        MitControl(motor_writers_.at(i), control_command);
        // TEST: failed to write bug.
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }

    } else if (mode_ == "slave_arm") {
      if (init_joint_soft_limit_) {
        // 保证各关节在软限位范围内
        if (init_trajectory_planning_) {
          AINFO << "init soft joint limit, wait a moment.";
          init_trajectory_planning_ = false;
          // 轨迹规划
          std::vector<double> init_joint_position = current_joint_position;
          std::vector<double> init_joint_velocity = current_joint_velocity;
          std::vector<double> init_joint_acc(7, 0);
          std::vector<double> end_joint_position(target_joint_position_.begin(),
                                                 target_joint_position_.end());
          if (load_end_motor_) {
            end_joint_position.push_back(target_gripper_joint_position_);
          }
          std::vector<double> end_joint_velocity(7, 0);
          std::vector<double> end_joint_acc(7, 0);
          double time = 0.8;
          arm_control_index_ = 0;
          arm_control_trajectory_.clear();

          // 5次多项式插值
          arm_control_trajectory_ =
              trajectory_generator_->GenerateQuinticDiscreteTrajectory(
                  {init_joint_position, init_joint_velocity, init_joint_acc},
                  {end_joint_position, end_joint_velocity, end_joint_acc}, time,
                  1.0 / control_hz);
          // AINFO << "time : " << time << " , time_step: " << 1.0 / control_hz
          //       << " ,size: " << arm_control_trajectory_.size();
        }

        if (arm_control_index_ < arm_control_trajectory_.size()) {
          // 轨迹跟踪
          std::vector<ControlCommand> control_command_vec =
              arm_control_trajectory_[arm_control_index_];

          for (int j = 0; j < motor_writers_.size(); j++) {
            auto motor = motor_writers_.at(j);
            ControlCommand control_command = control_command_vec[j];
            // kp
            control_command.kp = motor->MitKp();
            // kd
            control_command.kd = motor->MitKd();
            // torque
            if (j < 6) {
              control_command.torque = feed_forward_torque[j];
            } else {
              // control_command.torque = kdl_solver_.GripperTorqueCompensation(
              //     current_joint_velocity.at(j), arm_end_type_);
              control_command.torque = 0;
            }

            MitControl(motor, control_command, false);
            // TEST: failed to write bug.
            std::this_thread::sleep_for(std::chrono::microseconds(100));
          }
          arm_control_index_++;
        } else {
          init_joint_soft_limit_ = false;
        }

      } else {
        {
          std::lock_guard<std::mutex> lock(arm_control_mutux_);
          if (synchronous_master_arm_) {
            synchronous_master_arm_ = false;
            arm_control_index_ = 0;
            arm_control_trajectory_.clear();
            // 第一次从臂与主臂低速同步位置, 插值处理速度
            std::vector<double> init_joint_position = current_joint_position;
            std::vector<double> init_joint_velocity = current_joint_velocity;
            std::vector<double> init_joint_acc(7, 0);
            std::vector<double> end_joint_position =
                follow_target_joint_position_;
            std::vector<double> end_joint_velocity(7, 0);
            std::vector<double> end_joint_acc(7, 0);

            double max_delta_position = 0;
            for (int i = 0; i < current_joint_position.size(); ++i) {
              double delta_position = std::fabs(end_joint_position.at(i) -
                                                current_joint_position.at(i));
              if (delta_position > max_delta_position) {
                max_delta_position = delta_position;
              }
            }
            double time = max_delta_position * 2;

            // 5次多项式插值
            arm_control_trajectory_ =
                trajectory_generator_->GenerateQuinticDiscreteTrajectory(
                    {init_joint_position, init_joint_velocity, init_joint_acc},
                    {end_joint_position, end_joint_velocity, end_joint_acc},
                    time, 1.0 / control_hz);
            AINFO << "slave arm start to synchronous master arm joint "
                     "position, wait "
                  << time << " sec";
          }

          if (arm_control_index_ < arm_control_trajectory_.size()) {
            // 轨迹跟踪
            std::vector<ControlCommand> control_command_vec =
                arm_control_trajectory_[arm_control_index_];

            for (int j = 0; j < motor_writers_.size(); j++) {
              auto motor = motor_writers_.at(j);
              ControlCommand control_command = control_command_vec[j];
              // kp
              control_command.kp = motor->FollowMitKp();
              // kd
              control_command.kd = motor->FollowMitKd();
              // torque
              if (j < 6) {
                control_command.torque = feed_forward_torque[j];
              } else {
                // control_command.torque =
                // kdl_solver_.GripperTorqueCompensation(
                //     current_joint_velocity[j], arm_end_type_);
                control_command.torque = 0;
              }

              MitControl(motor, control_command);
              // TEST: failed to write bug.
              std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            arm_control_index_++;

            if (arm_control_index_ == arm_control_trajectory_.size()) {
              AINFO << "synchronous completed, start remote control.";
            }

          } else {
            // 插值结束后,直接快速跟踪主臂
            for (int i = 0; i < motor_writers_.size(); i++) {
              auto motor = motor_writers_.at(i);
              ControlCommand control_command;
              if (i < 6) {
                // arm control
                // position
                control_command.position = follow_target_joint_position_.at(i);
                // velocity
                control_command.velocity = 0;
                // kp
                control_command.kp = motor->FollowMitKp();
                // kd
                control_command.kd = motor->FollowMitKd();
                // torque
                control_command.torque = feed_forward_torque[i];

              } else {
                // gripper control
                double gripper_torque = current_joint_torque.at(i);
                if (gripper_torque < -2.9 &&
                    follow_target_joint_position_.at(i) <
                        current_joint_position.at(i)) {
                  // 控制指令还在向内夹(比当前位置小)， 就保持固定扭矩输出
                  control_command.position = 0;
                  control_command.velocity = 0;
                  control_command.kp = 0;
                  control_command.kd = 0;
                  control_command.torque = -3;

                } else {
                  control_command.position =
                      follow_target_joint_position_.at(i);
                  control_command.velocity = 0;
                  control_command.kp = motor->FollowMitKp();
                  control_command.kd = motor->FollowMitKd();
                  // control_command.torque =
                  //     kdl_solver_.GripperTorqueCompensation(
                  //         current_joint_velocity[i], arm_end_type_);
                  control_command.torque = 0;
                }
              }

              MitControl(motor, control_command);
              // TEST: failed to write bug
              std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
          }
        }
      }

    } else if (mode_ == "NRT_JOINT_POSITION") {
      // normal control arm: no real time joint position control.
      // trajectory interpolation start state and end state
      std::vector<double> init_joint_position = current_joint_position;
      std::vector<double> init_joint_velocity = current_joint_velocity;
      std::vector<double> init_joint_acc(7, 0);

      if (init_joint_soft_limit_) {
        // 保证各关节在软限位范围内
        if (init_trajectory_planning_) {
          AINFO << "init soft joint limit, wait a moment.";
          init_trajectory_planning_ = false;
          // 轨迹规划
          std::vector<double> end_joint_position(target_joint_position_.begin(),
                                                 target_joint_position_.end());
          if (load_end_motor_) {
            end_joint_position.push_back(target_gripper_joint_position_);
          }
          std::vector<double> end_joint_velocity(7, 0);
          std::vector<double> end_joint_acc(7, 0);
          double time = 0.8;
          arm_control_index_ = 0;
          arm_control_trajectory_.clear();

          // 5次多项式插值
          arm_control_trajectory_ =
              trajectory_generator_->GenerateQuinticDiscreteTrajectory(
                  {init_joint_position, init_joint_velocity, init_joint_acc},
                  {end_joint_position, end_joint_velocity, end_joint_acc}, time,
                  1.0 / control_hz);
        }

        if (arm_control_index_ < arm_control_trajectory_.size()) {
          // 轨迹跟踪
          std::vector<ControlCommand> control_command_vec =
              arm_control_trajectory_[arm_control_index_];

          for (int j = 0; j < motor_writers_.size(); j++) {
            auto motor = motor_writers_.at(j);
            ControlCommand control_command = control_command_vec[j];
            // kp
            control_command.kp = motor->MitKp();
            // kd
            control_command.kd = motor->MitKd();
            // torque
            if (j < 6) {
              control_command.torque = feed_forward_torque[j];
            } else {
              // control_command.torque = kdl_solver_.GripperTorqueCompensation(
              //     current_joint_velocity[j], arm_end_type_);
              control_command.torque = 0;
            }

            MitControl(motor, control_command, false);
            // TEST: failed to write bug.
            std::this_thread::sleep_for(std::chrono::microseconds(100));
          }
          arm_control_index_++;
        } else {
          init_joint_soft_limit_ = false;
        }

      } else {
        {
          // control arm
          std::lock_guard<std::mutex> lock(arm_control_mutux_);
          std::vector<double> end_joint_position(target_joint_position_.begin(),
                                                 target_joint_position_.end());
          std::vector<double> end_joint_velocity(6, 0);
          std::vector<double> end_joint_acc(6, 0);

          // 轨迹规划仅在目标位置发生变化时执行
          if (new_arm_target_position_) {
            new_arm_target_position_ = false;
            // 以上一时刻下发的位置 作为初始位置
            for (int i = 0; i < 6; i++) {
              init_joint_position.at(i) = last_control_joint_position_.at(i);
              init_joint_velocity.at(i) = last_control_joint_velocity_.at(i);
              // init_joint_acc.at(i) = last_control_joint_acc_.at(i);
            }

            arm_control_index_ = 0;
            arm_control_trajectory_.clear();

            // NOTE(known-limitation): add max vel and acc and jerk limit. 见 docs/metal_sdk_known_limitations.md
            double time;
            double max_delta_position = 0;
            for (int i = 0; i < 6; ++i) {
              double delta_position = std::fabs(end_joint_position.at(i) -
                                                current_joint_position.at(i));
              if (delta_position > max_delta_position) {
                max_delta_position = delta_position;
              }
            }
            if (has_last_target_position_time_) {
              auto time_result = trajectory_time_allocator_.SolveQuinticTime(
                  max_delta_position, 1, has_last_target_position_time_);
              // if (time_result.T > arm_control_dt_ + 4.0 / control_hz) {
              //   AINFO << "arm limit";
              // } else {
              //   AINFO << "arm no limit";
              // }
              // AINFO << "trajectory_time_allocator_time: " << time_result.T
              //       << " , track hz: " << arm_control_dt_ + 4.0 / control_hz;
              time =
                  std::fmax(arm_control_dt_ + 4.0 / control_hz, time_result.T);
              for (int i = 0; i < end_joint_position.size(); i++) {
                double delta_position =
                    end_joint_position[i] - init_joint_position[i];
                end_joint_velocity[i] = delta_position / time;
              }
            } else {
              auto time_result = trajectory_time_allocator_.SolveQuinticTime(
                  max_delta_position, joint_speed_scale_,
                  has_last_target_position_time_);
              has_last_target_position_time_ = true;
              time = time_result.T;
            }

            // 5次多项式插值
            arm_control_trajectory_ =
                trajectory_generator_->GenerateQuinticDiscreteTrajectory(
                    {init_joint_position, init_joint_velocity, init_joint_acc},
                    {end_joint_position, end_joint_velocity, end_joint_acc},
                    time, 1.0 / control_hz);
            // arm_control_trajectory_ =
            //     trajectory_generator_->LinearInterpolation(
            //         init_joint_position, end_joint_position, time,
            //         1.0 / control_hz);
            // AINFO << "time : " << time << " , time_step: " << 1.0 /
            // control_hz
            //       << " ,size: " << arm_control_trajectory_.size();
          }

          if (arm_control_index_ < arm_control_trajectory_.size()) {
            // 轨迹跟踪
            // AINFO << "arm_control_index_: " << arm_control_index_ + 1;
            std::vector<ControlCommand> control_command_vec =
                arm_control_trajectory_[arm_control_index_];

            // 补偿末端负载带来的力矩
            // kdl_solver_.ComputePayloadCompensation(
            //     current_joint_position, current_joint_torque,
            //     payload_torque);
            for (int j = 0; j < 6; j++) {
              auto motor = motor_writers_.at(j);
              ControlCommand control_command = control_command_vec[j];
              // kp
              control_command.kp = motor->MitKp();
              // kd
              control_command.kd = motor->MitKd();
              // torque
              control_command.torque = feed_forward_torque[j];

              MitControl(motor, control_command);
              // record last control command
              last_control_joint_position_.at(j) = control_command.position;
              last_control_joint_velocity_.at(j) = control_command.velocity;
              last_control_joint_acc_.at(j) = control_command.acc;
              // TEST: failed to write bug.
              std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            arm_control_index_++;

          } else {
            // 轨迹完成，发送最后一个点保持稳定
            // AINFO << "Send target joint position.";
            for (int j = 0; j < 6; j++) {
              auto motor = motor_writers_.at(j);
              ControlCommand control_command;
              control_command.position = target_joint_position_.at(j);
              // TEST: target velocity not 0
              // if (!arm_control_trajectory_.empty()) {
              //   control_command.velocity =
              //       arm_control_trajectory_.back().at(j).velocity;
              // } else {
              //   control_command.velocity = 0;
              // }
              control_command.velocity = 0;
              control_command.kp = motor->MitKp();
              control_command.kd = motor->MitKd();
              control_command.torque = feed_forward_torque[j];

              MitControl(motor, control_command);
              // record last control command
              last_control_joint_position_.at(j) = control_command.position;
              last_control_joint_velocity_.at(j) = control_command.velocity;
              last_control_joint_acc_.at(j) = 0;
              // TEST: failed to write bug.
              std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
          }
        }

        if (load_end_motor_) {
          {
            // control gripper
            std::lock_guard<std::mutex> lock(gripper_control_mutux_);
            if (new_gripper_target_position_) {
              new_gripper_target_position_ = false;
              gripper_control_index_ = 0;
              gripper_control_trajectory_.clear();

              double gripper_init_position = last_control_gripper_position_;
              double gripper_init_vel = last_control_gripper_velocity_;
              double gripper_init_acc = last_control_gripper_acc_;

              // add max vel and acc and jerk limit
              // AINFO << "gripper_init_position: " << gripper_init_position
              //       << " , target_gripper_joint_position_: "
                    // << target_gripper_joint_position_;
              double gripper_delta_postion =
                  target_gripper_joint_position_ - gripper_init_position;
              double time = 0;
              double gripper_end_vel = 0;
              if (has_last_gripper_position_time_) {
                auto time_result = trajectory_time_allocator_.SolveQuinticTime(
                    gripper_delta_postion, 1, has_last_gripper_position_time_,
                    true);
                // AINFO << "trajectory_time_allocator_time: " << time_result.T
                //       << " , gripper_control_dt_: "
                //       << gripper_control_dt_ + 4.0 / control_hz;
                // if (time_result.T > gripper_control_dt_ + 4.0 / control_hz) {
                //   AINFO << "gripper limit";
                // } else {
                //   AINFO << "gripper no limit";
                // }

                time = std::fmax(gripper_control_dt_ + 4.0 / control_hz,
                                 time_result.T);
                gripper_end_vel = gripper_delta_postion / time;
              } else {
                auto time_result = trajectory_time_allocator_.SolveQuinticTime(
                    gripper_delta_postion, gripper_speed_scale_,
                    has_last_gripper_position_time_, true);
                has_last_gripper_position_time_ = true;
                time = time_result.T;
              }

              gripper_control_trajectory_ =
                  trajectory_generator_
                      ->GenerateSingleQuinticDiscreteTrajectory(
                          {gripper_init_position, gripper_init_vel,
                           gripper_init_acc},
                          {target_gripper_joint_position_, gripper_end_vel, 0},
                          time, 1.0 / control_hz);
            }

            if (gripper_control_index_ < gripper_control_trajectory_.size()) {
              // 轨迹跟踪
              // AINFO << "gripper_control_index_: " << gripper_control_index_ +
              auto motor = motor_writers_.at(6);
              ControlCommand control_command =
                  gripper_control_trajectory_.at(gripper_control_index_);
              // kp
              control_command.kp = motor->MitKp();
              // kd
              control_command.kd = motor->MitKd();
              // torque
              // control_command.torque = kdl_solver_.GripperTorqueCompensation(
              //     current_joint_velocity[6], arm_end_type_);
              control_command.torque = 0;

              MitControl(motor, control_command);
              last_control_gripper_position_ = control_command.position;
              last_control_gripper_velocity_ = control_command.velocity;
              last_control_gripper_acc_ = control_command.acc;
              gripper_control_index_++;
            } else {
              // send gripper target position
              auto motor = motor_writers_.at(6);
              ControlCommand control_command;
              control_command.position = target_gripper_joint_position_;
              control_command.velocity = 0;
              // kp
              control_command.kp = motor->MitKp();
              // kd
              control_command.kd = motor->MitKd();
              // torque
              // control_command.torque = kdl_solver_.GripperTorqueCompensation(
              //     current_joint_velocity[6], arm_end_type_);
              control_command.torque = 0;

              MitControl(motor, control_command);
              last_control_gripper_position_ = control_command.position;
              last_control_gripper_velocity_ = control_command.velocity;
              last_control_gripper_acc_ = 0;
            }
          }
        }
      }

    } else {
      AERROR << "Invalid control mode: " << mode_;
      stop_flag_.store(true);
      break;
    }

    rate.sleep();

    // auto ff_time = std::chrono::high_resolution_clock::now();
    // AINFO << "loop time: "
    //       << std::chrono::duration<double, std::milli>(ff_time - start_time)
    //              .count()
    //       << " ms";
  }

  AINFO << "Can writer thread finish";
}

}  // namespace metal
}  // namespace makermods
