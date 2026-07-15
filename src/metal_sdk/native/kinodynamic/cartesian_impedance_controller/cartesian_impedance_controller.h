#pragma once

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <vector>

class CartesianImpedanceController {
public:
  CartesianImpedanceController(const KDL::Chain &chain);

  void SetGains(const KDL::Vector &stiffness, const KDL::Vector &damping);

  // 主要控制函数
  std::vector<double> ComputeTorque(const KDL::JntArray &q,
                                    const KDL::JntArray &dq,
                                    const KDL::Frame &x_des,
                                    const KDL::Twist &dx_des);

private:
  KDL::Chain chain_;
  KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainJntToJacSolver jac_solver_;

  KDL::Vector stiffness_; // 末端刚度（3位置 + 3姿态）
  KDL::Vector damping_;   // 末端阻尼
};