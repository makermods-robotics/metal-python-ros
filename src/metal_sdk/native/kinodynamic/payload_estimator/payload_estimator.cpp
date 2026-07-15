#include "payload_estimator.h"

#include <common/log.h>

#include <Eigen/Dense>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>

namespace makermods {
namespace metal {

// === 工具函数 ===
namespace {
inline Eigen::Vector3d ToEigen(const KDL::Vector &v) {
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

inline Eigen::VectorXd MakeGravityWrench(const KDL::Vector &g) {
  Eigen::VectorXd wrench(6);
  wrench.setZero();
  wrench.segment<3>(0) = ToEigen(g);  // 只赋值前三维力，后三维力矩为 0
  return wrench;
}
}  // namespace

// === 构造与初始化 ===
PayloadEstimator::PayloadEstimator(const KDL::Chain &chain)
    : chain_(chain),
      gravity_(0.0, 0.0, -9.81),
      mass_(1.0),
      lambda_(0.98),
      P_(1000.0) {
  dyn_solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
}

void PayloadEstimator::Init(double lambda, double init_mass) {
  lambda_ = lambda;
  mass_ = init_mass;
  P_ = 1000.0;
}

// === RLS 质量估计 ===
void PayloadEstimator::UpdateMassRLS(const KDL::JntArray &q,
                                     const KDL::JntArray &dq,
                                     const KDL::JntArray &ddq,
                                     const KDL::JntArray &tau) {
  KDL::JntArray coriolis(q.rows()), gravity(q.rows());
  KDL::JntSpaceInertiaMatrix inertia(q.rows());
  // TODO: 待优化，避免重复计算前馈力矩
  dyn_solver_->JntToCoriolis(q, dq, coriolis);
  dyn_solver_->JntToGravity(q, gravity);
  dyn_solver_->JntToMass(q, inertia);

  KDL::JntArray tau_model(q.rows());
  for (unsigned int i = 0; i < q.rows(); ++i) {
    double inertia_term = 0.0;
    for (unsigned int j = 0; j < q.rows(); ++j) {
      inertia_term += inertia(i, j) * ddq(j);
    }
    tau_model(i) = inertia_term + coriolis(i) + gravity(i);
  }

  double tau_ext = 0.0;
  for (unsigned int i = 0; i < q.rows(); ++i) {
    tau_ext += tau(i) - tau_model(i);
  }

  KDL::Jacobian jacobian(q.rows());
  jac_solver_->JntToJac(q, jacobian);

  Eigen::VectorXd phi = jacobian.data.transpose() * MakeGravityWrench(gravity_);
  double phi_scalar = phi.sum();

  double K = P_ * phi_scalar / (lambda_ + phi_scalar * P_ * phi_scalar);
  mass_ = mass_ + K * (tau_ext - phi_scalar * mass_);
  P_ = (1 - K * phi_scalar) * P_ / lambda_;
}

// === 静态雅可比质量估计（静止情况下）===
double PayloadEstimator::EstimateMassJacobian(
    const KDL::JntArray &q, const KDL::JntArray &tau,
    const KDL::JntArray &gravity_tau) const {
  // KDL::JntArray gravity(q.rows());
  // dyn_solver_->JntToGravity(q, gravity);

  KDL::Jacobian jacobian(q.rows());
  jac_solver_->JntToJac(q, jacobian);

  Eigen::VectorXd tau_ext(q.rows());
  for (unsigned int i = 0; i < q.rows(); ++i) {
    tau_ext(i) = tau(i) - gravity_tau(i);
  }

  Eigen::VectorXd phi = jacobian.data.transpose() * MakeGravityWrench(gravity_);
  double numerator = tau_ext.transpose() * phi;

  double denominator = gravity_.x() * gravity_.x() +
                       gravity_.y() * gravity_.y() +
                       gravity_.z() * gravity_.z();

  return numerator / denominator;
}

// === 获取当前估计质量 ===
double PayloadEstimator::GetEstimatedMass() const { return mass_; }

// === 计算补偿力矩（用于前馈）===
bool PayloadEstimator::ComputePayloadCompensation(
    const KDL::JntArray &q, KDL::JntArray &tau_comp) const {
  if (q.rows() != chain_.getNrOfJoints()) {
    return false;
  }

  KDL::Jacobian jacobian(q.rows());
  if (jac_solver_->JntToJac(q, jacobian) < 0) {
    return false;
  }

  // 先试用基于雅克比矩阵的负载估算方法
  //   double mass_jac = EstimateMassJacobian(q, tau_);
  Eigen::VectorXd Fg = MakeGravityWrench(gravity_) * mass_;
  Eigen::VectorXd tau_eigen = jacobian.data.transpose() * Fg;

  tau_comp.resize(q.rows());
  for (unsigned int i = 0; i < q.rows(); ++i) {
    tau_comp(i) = tau_eigen(i);
  }

  return true;
}

// 适配当前算法的函数接口
bool PayloadEstimator::ComputePayloadCompensation(
    const std::vector<double> &q, const std::vector<double> &cur_joint_tau,
    const std::vector<double> &gravity_torque_kdl_,
    std::vector<double> &tau_comp) const {
  // 检查输入维度
  if (q.size() != chain_.getNrOfJoints()) {
    return false;
  }

  // 将 std::vector 转换为 KDL::JntArray
  KDL::JntArray q_kdl(q.size());
  for (size_t i = 0; i < q.size(); ++i) {
    q_kdl(i) = q[i];
  }

  // 计算雅可比矩阵
  KDL::Jacobian jacobian(q.size());
  if (jac_solver_->JntToJac(q_kdl, jacobian) < 0) {
    return false;
  }

  // TODO:先使用雅可比矩阵估算负载的方法
  // 将 std::vector 转换为 KDL::JntArray
  KDL::JntArray tau_kdl(cur_joint_tau.size());
  for (size_t i = 0; i < cur_joint_tau.size(); ++i) {
    tau_kdl(i) = cur_joint_tau[i];
  }
  KDL::JntArray gravity_kdl(gravity_torque_kdl_.size());
  for (size_t i = 0; i < gravity_torque_kdl_.size(); ++i) {
    gravity_kdl(i) = gravity_torque_kdl_[i];
  }

  double mass_jac = EstimateMassJacobian(q_kdl, tau_kdl, gravity_kdl);
  // debug
  AINFO << "mass_jac: " << mass_jac;

  // 构造重力力矩
  Eigen::VectorXd Fg = MakeGravityWrench(gravity_) * mass_jac;
  Eigen::VectorXd tau_eigen = jacobian.data.transpose() * Fg;

  // 写入输出
  tau_comp.resize(q.size());
  for (size_t i = 0; i < q.size(); ++i) {
    tau_comp[i] = tau_eigen(i);
  }

  return true;
}

// TODO:下一次指令下发前，检查是否发生碰撞，若发生碰撞，则保持当前位置
// if (IsCollisionDetected(control_command_vec)) {
//   // 1.方案一：更新目标位置为当前位置
//   std::vector<double> joint_positions = GetJointPosition();
//   if (joint_positions.size() >= 6) {
//     std::copy_n(joint_positions.begin(), 6,
//                 target_joint_position_.begin());
//   } else {
//     AERROR << "GetJointPosition() 返回的元素少于6个";
//   }
//   std::vector<ControlCommand> stop_control_command;
//   // 2.方案二：保持当前位置
//   KeepCurrentPostion(GetJointPosition(), is_mit_mode_,
//                      stop_control_command);
//   for (int i = 0; i < stop_control_command.size(); i++) {
//     motor_writers_.at(i)->MitControl(write_frame,
//                                      stop_control_command[i]);
//     if (!WriteCanFrame(write_frame, socket_.load())) {
//       AERROR << "Failed to write can frame to device: "
//              << motor_writers_.at(i)->name();
//     }
//     std::this_thread::sleep_for(std::chrono::microseconds(200));
//   }
//   AERROR << "Collision detected! keep current position!";
//   // 退出插值轨迹for循环
//   break;
// }

}  // namespace metal
}  // namespace makermods
