#pragma once

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <memory>

namespace makermods {
namespace metal {

class PayloadEstimator {
 public:
  explicit PayloadEstimator(const KDL::Chain &chain);

  // 初始化估计器（可调遗忘因子与初始质量）
  void Init(double lambda = 0.98, double init_mass = 1.0);

  // 使用反演动力学 + RLS 更新估计的质量
  void UpdateMassRLS(const KDL::JntArray &q, const KDL::JntArray &dq,
                     const KDL::JntArray &ddq, const KDL::JntArray &tau);

  // 使用雅可比矩阵估计质量（静止工况）
  double EstimateMassJacobian(const KDL::JntArray &q, const KDL::JntArray &tau,
                              const KDL::JntArray &gravity_tau) const;

  // 获取当前估计质量（单位：kg）
  double GetEstimatedMass() const;

  // 返回末端负载质量引起的补偿力矩
  bool ComputePayloadCompensation(const KDL::JntArray &q,
                                  KDL::JntArray &tau_comp) const;
  //   适配参数接口
  bool ComputePayloadCompensation(
      const std::vector<double> &q, const std::vector<double> &cur_joint_tau,
      const std::vector<double> &gravity_torque_kdl_,
      std::vector<double> &tau_comp) const;

 private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  double lambda_;  // 遗忘因子（控制RLS记忆性）
  double mass_;    // 当前估计的质量值
  double P_;       // 协方差（估计可信度）

  KDL::Vector gravity_;  // 重力方向向量，默认 (0, 0, -9.81)
};

}  // namespace metal
}  // namespace makermods
