#include "cartesian_impedance_controller.h"
#include <kdl/utilities/error.h>
#include <kdl/utilities/svd_eigen_HH.hpp>

CartesianImpedanceController::CartesianImpedanceController(
    const KDL::Chain &chain)
    : chain_(chain), fk_solver_(chain), jac_solver_(chain) {}

void CartesianImpedanceController::SetGains(const KDL::Vector &stiffness,
                                            const KDL::Vector &damping) {
  stiffness_ = stiffness;
  damping_ = damping;
}

std::vector<double> CartesianImpedanceController::ComputeTorque(
    const KDL::JntArray &q, const KDL::JntArray &dq, const KDL::Frame &x_des,
    const KDL::Twist &dx_des) {

  // Step 1: 正向运动学计算当前末端位姿
  KDL::Frame x_now;
  fk_solver_.JntToCart(q, x_now);

  // Step 2: 计算位姿误差
  KDL::Twist x_err =
      KDL::diff(x_now, x_des); // Twist: (dx, dy, dz, droll, dpitch, dyaw)

  // Step 3: 雅可比矩阵
  KDL::Jacobian J(chain_.getNrOfJoints());
  jac_solver_.JntToJac(q, J);

  // Step 4: 虚拟阻抗模型计算末端期望力
  // KDL::Twist dx_now = J * dq; // 不支持直接 J * dq
  KDL::Twist dx_now;
  MultiplyJacobian(J, dq, dx_now); // 将结果直接写入 dx_now

  KDL::Twist dx_err = dx_des - dx_now;

  KDL::Wrench F;
  for (int i = 0; i < 3; ++i) {
    F.force[i] = stiffness_[i] * x_err.vel[i] + damping_[i] * dx_err.vel[i];
    F.torque[i] =
        stiffness_[i + 3] * x_err.rot[i] + damping_[i + 3] * dx_err.rot[i];
  }

  // Step 5: 映射为关节力矩: τ = Jᵀ * F
  std::vector<double> tau(chain_.getNrOfJoints(), 0.0);
  for (unsigned int j = 0; j < chain_.getNrOfJoints(); ++j) {
    for (unsigned int i = 0; i < 6; ++i) {
      tau[j] += J(i, j) * F[i];
    }
  }

  return tau;
}