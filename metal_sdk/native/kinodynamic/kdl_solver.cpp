#include "kdl_solver.h"

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <urdf/model.h>
#include <urdf_model/model.h>
#include <urdf_model_state/model_state.h>
#include <urdf_parser/urdf_parser.h>

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "common/log.h"

namespace makermods {
namespace metal {

int sign(double x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;  // x == 0
}

KdlSolver::KdlSolver(const std::string& urdf_file, int arm_end_type)
    : urdf_file_(urdf_file), arm_end_type_(arm_end_type) {}

/**
 * @brief load URDF, build the KDL chain/solvers and the TRAC-IK/payload
 * estimator instances. Must be called before any other public method.
 */
bool KdlSolver::Init() {
  // 加载 URDF 模型
  urdf::Model model;
  if (!model.initFile(urdf_file_)) {
    AERROR << "Failed to parse URDF";
    return false;
  }

  // 从 URDF 构建 KDL Tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    AERROR << "Failed to construct KDL tree";
    return false;
  }

  // 从 KDL Tree 提取 KDL Chain
  if (arm_end_type_ == 0) {
    if (!kdl_tree.getChain("base_link", "Link6", kdl_chain_)) {
      AERROR << "Failed to extract chain from base_link to Link6";
      return false;
    }
  } else if (arm_end_type_ == 1) {
    if (!kdl_tree.getChain("base_link", "Link6", kdl_chain_)) {
      AERROR << "Failed to extract chain from base_link to Link6";
      return false;
    }
  } else if (arm_end_type_ == 2) {
    if (!kdl_tree.getChain("base_link", "Link6", kdl_chain_)) {
      AERROR << "Failed to extract chain from base_link to Link6";
      return false;
    }
  } else if (arm_end_type_ == 3) {
    if (!kdl_tree.getChain("base_link", "Link6", kdl_chain_)) {
      AERROR << "Failed to extract chain from base_link to Link6 [gripper]";
      return false;
    }
  } else {
    AERROR << "arm_end_type must be 0, 1, 2 or 3";
    return false;
  }

  // 获取关节数量
  KDL::JntArray joint_min, joint_max;
  joint_min.resize(kdl_chain_.getNrOfJoints());
  joint_max.resize(kdl_chain_.getNrOfJoints());

  for (int i = 0; i < kdl_chain_.segments.size(); ++i) {
    auto segment = kdl_chain_.segments.at(i);
    const KDL::Joint& joint = segment.getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }

    const std::string& joint_name = joint.getName();
    auto it = model.joints_.find(joint_name);
    if (it == model.joints_.end()) {
      AERROR << "Joint " << joint_name << " not found in URDF!";
      return false;
    }

    const auto& urdf_joint = it->second;

    if ((urdf_joint->type == urdf::Joint::REVOLUTE ||
         urdf_joint->type == urdf::Joint::PRISMATIC) &&
        urdf_joint->limits) {
      joint_min(i) = urdf_joint->limits->lower;
      joint_max(i) = urdf_joint->limits->upper;
    }
  }

  KDL::Vector gravity(0.0, 0.0, -9.81);  // 默认重力方向
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  dyn_solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity);
  // 初始化 TracIK
  tracik_solver_.reset(new TRAC_IK::TRAC_IK(kdl_chain_, joint_min, joint_max,
                                            100, 1e-5, TRAC_IK::Speed));

  // 新增负载估计器
  payload_estimator_ = std::make_unique<PayloadEstimator>(kdl_chain_);

  return true;
}

/**
 * @brief forward kinematics: joint positions -> end effector pose (xyz+rpy).
 */
bool KdlSolver::FkSolver(const std::array<double, 6>& joint_positions,
                         std::array<double, 6>& end_pose) {
  KDL::Frame out_pose;

  KDL::JntArray q(joint_positions.size());

  for (size_t i = 0; i < joint_positions.size(); ++i) {
    q(i) = joint_positions[i];
  }

  if (fk_solver_->JntToCart(q, out_pose) < 0) {
    AERROR << "fk_solver Failed !";
    return false;
  }

  end_pose[0] = out_pose.p.x();
  end_pose[1] = out_pose.p.y();
  end_pose[2] = out_pose.p.z();
  double roll, pitch, yaw;
  out_pose.M.GetRPY(roll, pitch, yaw);
  end_pose[3] = roll;
  end_pose[4] = pitch;
  end_pose[5] = yaw;

  return true;
}

/**
 * @brief inverse kinematics via TRAC-IK, seeded from init_joint_positions.
 * @return false if no reachable solution is found.
 */
bool KdlSolver::IkSolverWithTracIK(
    const std::array<double, 6>& arm_end_pose,
    const std::array<double, 6>& init_joint_positions,
    std::array<double, 6>& joint_positions) {
  // 构造目标末端位姿
  KDL::Frame desired_pose(
      KDL::Rotation::RPY(arm_end_pose[3], arm_end_pose[4], arm_end_pose[5]),
      KDL::Vector(arm_end_pose[0], arm_end_pose[1], arm_end_pose[2]));

  KDL::JntArray q_init(kdl_chain_.getNrOfJoints());
  for (int i = 0; i < 6; ++i) {
    q_init(i) = init_joint_positions[i];
  }

  KDL::JntArray q_result(kdl_chain_.getNrOfJoints());

  int rc = SafeCartToJnt(q_init, desired_pose, q_result, 5000);

  if (rc < 0) {
    AERROR << "[TracIK] Failed to solve IK, please input reachable pose.";
    return false;
  }

  for (int i = 0; i < q_result.rows(); ++i) {
    joint_positions[i] = q_result(i);
  }

  return true;
}

/**
 * @brief run TRAC-IK's CartToJnt in a forked child process with a timeout,
 * so a hung/divergent IK solve cannot block the caller indefinitely.
 */
int KdlSolver::SafeCartToJnt(const KDL::JntArray& q_init,
                             const KDL::Frame& desired_pose,
                             KDL::JntArray& q_result, int timeout_sec) {
  int pipefd[2];
  if (pipe(pipefd) == -1) {
    AERROR << "[TracIK] Pipe creation failed.";
    return -1;
  }

  pid_t pid = fork();
  if (pid < 0) {
    AERROR << "[TracIK] Fork failed.";
    return -1;
  }

  if (pid == 0) {
    // 子进程：运行 IK
    close(pipefd[0]);  // 关闭读端

    // AWARN << "[TracIK] Running IK...";

    int rc = tracik_solver_->CartToJnt(q_init, desired_pose, q_result);

    // AWARN << "[TracIK] IK finished.";

    // 写入结果
    write(pipefd[1], &rc, sizeof(rc));
    if (rc >= 0) {
      for (int i = 0; i < q_result.rows(); ++i) {
        double val = q_result(i);
        write(pipefd[1], &val, sizeof(val));
      }
    }

    close(pipefd[1]);
    _exit(0);  // 子进程安全退出
  }

  // 父进程
  close(pipefd[1]);  // 关闭写端

  fd_set set;
  FD_ZERO(&set);
  FD_SET(pipefd[0], &set);

  struct timeval timeout;
  timeout.tv_sec = 0;             // 秒
  timeout.tv_usec = timeout_sec;  // 微妙

  int rv = select(pipefd[0] + 1, &set, NULL, NULL, &timeout);
  if (rv <= 0) {  // -1 错误 或 0 超时
    if (rv == 0) {
      AERROR << "[TracIK] IK solving timeout (>" << timeout_sec/1000
             << "s). Killing child.";
    } else {
      AERROR << "[TracIK] select() error.";
    }
    kill(pid, SIGKILL);
    waitpid(pid, NULL, 0);
    close(pipefd[0]);
    return -1;
  }

  // 有数据可读
  int rc;
  if (read(pipefd[0], &rc, sizeof(rc)) != sizeof(rc)) {
    AERROR << "[TracIK] Failed to read result.";
    waitpid(pid, NULL, 0);
    close(pipefd[0]);
    return -1;
  }

  if (rc >= 0) {
    for (int i = 0; i < q_result.rows(); ++i) {
      double val;
      read(pipefd[0], &val, sizeof(val));
      q_result(i) = val;
    }
  }

  close(pipefd[0]);
  waitpid(pid, NULL, 0);  // 回收子进程
  return rc;
}

/**
 * @brief compute feedforward joint torque (gravity + coriolis + friction)
 * for the current joint state.
 */
bool KdlSolver::FeedforwardTorqueCompensation(
    const std::vector<double>& current_joint_position,
    const std::vector<double>& current_joint_velocity,
    std::vector<double>& feed_forward_torque) {
  // 先清空
  feed_forward_torque.clear();

  auto joint_position = current_joint_position;
  auto joint_velocity = current_joint_velocity;

  if (current_joint_position.size() == 7) {
    joint_position.pop_back();
    joint_velocity.pop_back();
  }

  std::vector<double> gravity_torque_coe({1.2, 1.15, 1.1, 1.15, 1.0, 1.0});
  std::vector<double> coriolis_torque_coe({1.1, 1.15, 1.0, 1.0, 1.1, 1.1});
  // compute gravity torque
  std::vector<double> gravity_torque;
  if (!GravityCompensation(joint_position, gravity_torque)) {
    return false;
  }

  // compute coriolis torque
  std::vector<double> coriolis_torque;
  if (!CoriolisTorque(joint_position, joint_velocity, coriolis_torque)) {
    return false;
  }

  // compute friction torque
  std::vector<double> friction_torque;
  ComputeFriction(joint_velocity, friction_torque);

  // for (int i = 0; i < gravity_torque.size(); i++) {
  //   AINFO << "J" << i + 1 << " gravity: " << gravity_torque.at(i)
  //         << " , coriolis: " << coriolis_torque.at(i)
  //         << " ,friction: " << friction_torque.at(i) << " , total t: "
  //         << (gravity_torque.at(i) + coriolis_torque.at(i)) +
  //                friction_torque.at(i);
  // }

  for (int i = 0; i < joint_position.size(); i++) {
    double temp_ff_torque = gravity_torque_coe[i] * gravity_torque[i] +
                            coriolis_torque_coe[i] * coriolis_torque[i] +
                            friction_torque[i];
    feed_forward_torque.push_back(temp_ff_torque);
  }

  return true;
}

/**
 * @brief inertial joint torque M(q)*q_ddot.
 */
bool KdlSolver::InertiaTorque(const std::vector<double>& joint_positions,
                              const std::vector<double>& joint_acc,
                              std::vector<double>& inertia_torque) {
  size_t dof = joint_positions.size();
  KDL::JntArray q(dof), q_ddot(dof);
  for (size_t i = 0; i < dof; ++i) {
    q(i) = joint_positions[i];
    q_ddot(i) = joint_acc[i];
  }

  KDL::JntSpaceInertiaMatrix M(dof);
  if (dyn_solver_->JntToMass(q, M) < 0) {
    AERROR << "Failed to compute Mass Matrix";
    return false;
  }

  KDL::JntArray torque(dof);
  torque.data = M.data * q_ddot.data;

  inertia_torque.resize(dof);
  for (size_t i = 0; i < dof; ++i) {
    inertia_torque[i] = torque(i);
  }

  return true;
}

/**
 * @brief coriolis/centripetal joint torque via KDL::JntToCoriolis.
 */
bool KdlSolver::CoriolisTorque(const std::vector<double>& joint_positions,
                               const std::vector<double>& joint_vel,
                               std::vector<double>& coriolis_torque) {
  KDL::JntArray q(joint_positions.size());
  KDL::JntArray q_dot(joint_vel.size());
  for (size_t i = 0; i < joint_positions.size(); ++i) {
    q(i) = joint_positions[i];
    q_dot(i) = joint_vel[i];
  }
  // NOTE(known-limitation): 大模型说kdl的JntToCoriolis函数内部已经做了和q_ddot相乘的计算，
  // 输出的coriolis就是科氏力和向心力矩项，这里需要确认一下。见 known-limitations 文档
  KDL::JntArray coriolis(kdl_chain_.getNrOfJoints());
  if (dyn_solver_->JntToCoriolis(q, q_dot, coriolis) < 0) {
    AERROR << "Failed to compute Coriolis torque";
    return false;
  }

  coriolis_torque.resize(kdl_chain_.getNrOfJoints());
  for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    coriolis_torque[i] = coriolis(i);
  }

  return true;
}

/**
 * @brief gravity compensation joint torque (std::array overload).
 */
bool KdlSolver::GravityCompensation(
    const std::array<double, 6>& joint_positions,
    std::array<double, 6>& gravity_torque) {
  size_t dof = joint_positions.size();
  KDL::JntArray q(dof);
  for (size_t i = 0; i < dof; ++i) {
    q(i) = joint_positions[i];
  }

  KDL::JntArray gravity(dof);
  if (dyn_solver_->JntToGravity(q, gravity) < 0) {
    AERROR << "Failed to compute gravity torque";
    return false;
  }

  // gravity_torque.resize(dof);
  for (size_t i = 0; i < dof; ++i) {
    gravity_torque[i] = gravity(i);
  }
  return true;
}

/**
 * @brief gravity compensation joint torque (std::vector overload).
 */
bool KdlSolver::GravityCompensation(const std::vector<double>& joint_positions,
                                    std::vector<double>& gravity_torque) {
  size_t dof = joint_positions.size();
  KDL::JntArray q(dof);
  for (size_t i = 0; i < dof; ++i) {
    q(i) = joint_positions[i];
  }

  KDL::JntArray gravity(dof);
  if (dyn_solver_->JntToGravity(q, gravity) < 0) {
    AERROR << "Failed to compute gravity torque";
    return false;
  }

  gravity_torque.resize(dof);
  gravity_torque_kdl_.resize(dof);
  for (size_t i = 0; i < dof; ++i) {
    gravity_torque[i] = gravity(i);
    gravity_torque_kdl_[i] = gravity(i);
  }
  return true;
}

/**
 * @brief viscous/static friction torque estimate from joint velocity.
 */
void KdlSolver::ComputeFriction(const std::vector<double>& joint_velocity,
                                std::vector<double>& friction_torque) {
  friction_torque.resize(joint_velocity.size());
  std::vector<double> viscous_friction_coefficient(
      {0.15, 0.3025, 0.52, 0.42, 0.015, 0.015});
  // std::vector<double> coulomb_friction_coefficient({0, 0.1, 0.2, 0.3, 0.03,
  // 0});

  for (size_t i = 0; i < joint_velocity.size(); ++i) {
    double viscous_friction = 0;

    // 粘滞摩擦力（Viscous Friction）
    if (i == 2) {
      double vel = std::fmax(-0.8, std::fmin(0.8, joint_velocity.at(i)));
      viscous_friction = vel * viscous_friction_coefficient.at(i);

    } else if (i == 3) {
      double vel = std::fmax(-1.2, std::fmin(1.2, joint_velocity.at(i)));
      viscous_friction = vel * viscous_friction_coefficient.at(i);
    } else {
      viscous_friction =
          joint_velocity.at(i) * viscous_friction_coefficient.at(i);
    }

    // 库伦摩擦力
    // double coulomb_friction =
    //     sign(joint_velocity.at(i)) * coulomb_friction_coefficient.at(i);

    friction_torque.at(i) = viscous_friction;
  }
}

/**
 * @brief external/payload torque estimate; delegates to PayloadEstimator.
 */
bool KdlSolver::ComputePayloadCompensation(
    std::vector<double> current_joint_position,
    std::vector<double> current_joint_tau,
    std::vector<double>& tau_ext_torque) {
  if (!payload_estimator_->ComputePayloadCompensation(
          current_joint_position, current_joint_tau, gravity_torque_kdl_,
          tau_ext_torque)) {
    AERROR << "payload compensation failed";
    return false;
  }
  return true;
}

/**
 * @brief gripper friction torque compensation as a function of velocity.
 */
double KdlSolver::GripperTorqueCompensation(double velocity, int gripper_type) {
  double stop_torque = 0.06;
  double static_friction_torque = 0.03;
  if (velocity < 1e-5 && velocity > -1e-5) {
    
    return stop_torque;
  }

  static_friction_torque = sign(velocity) * static_friction_torque;
  // 粘滞摩擦力（Viscous Friction）
  double viscous_friction_coefficient = 0.01;
  double vel = std::fmax(-3, std::fmin(3, velocity));
  double viscous_friction = vel * viscous_friction_coefficient;

  return viscous_friction + static_friction_torque;
}

}  // namespace metal
}  // namespace makermods
