#include "polynomial_interpolation.h"

#include <array>
#include <cmath>
#include <vector>

#include "common/log.h"

namespace makermods {
namespace metal {

std::vector<std::vector<ControlCommand>>
PolynomialInterpolation::GenerateCubicDiscreteTrajectory(
    std::pair<JointArray, JointArray> init_state,
    std::pair<JointArray, JointArray> end_state, double time,
    double step) const {
  int joint_num = init_state.first.size();
  double planning_time = std::fabs(time);

  // 三次多项式 : q(t) = a + b*t + c*t^2 + d*t^3
  std::vector<double> coff_a;
  std::vector<double> coff_b;
  std::vector<double> coff_c;
  std::vector<double> coff_d;

  for (int i = 0; i < joint_num; ++i) {
    double init_pos = init_state.first[i];
    double init_vel = init_state.second[i];
    double end_pos = end_state.first[i];
    double end_vel = end_state.second[i];

    double a = init_pos;
    double b = init_vel;
    double c =
        (3 * (end_pos - init_pos) - (2 * init_vel + end_vel) * planning_time) /
        (planning_time * planning_time);
    double d =
        (2 * (init_pos - end_pos) + (init_vel + end_vel) * planning_time) /
        (planning_time * planning_time * planning_time);

    coff_a.push_back(a);
    coff_b.push_back(b);
    coff_c.push_back(c);
    coff_d.push_back(d);
  }

  std::vector<std::vector<ControlCommand>> result;
  std::vector<ControlCommand> cmd(joint_num);
  double step_count = step;
  while (step_count <= planning_time) {
    for (int i = 0; i < joint_num; ++i) {
      if (std::fabs(init_state.first[i] - end_state.first[i]) < 0.05) {
        cmd[i].position = end_state.first[i];
        cmd[i].velocity = 0;
      } else {
        // 插值位置
        cmd[i].position = coff_a[i] + coff_b[i] * step_count +
                          coff_c[i] * step_count * step_count +
                          coff_d[i] * step_count * step_count * step_count;
        // 速度计算
        cmd[i].velocity = coff_b[i] + 2 * coff_c[i] * step_count +
                          3 * coff_d[i] * step_count * step_count;
        // AINFO << "planning_time: " << step_count << " ,J" << i + 1
        //       << " position:" << cmd[i].position << " , velocity "
        //       << cmd[i].velocity;
      }
    }

    result.push_back(cmd);
    step_count += step;
  }

  // push end state
  for (int i = 0; i < joint_num; ++i) {
    cmd[i].position = end_state.first[i];
    cmd[i].velocity = end_state.second[i];
    AINFO << "last time: " << step_count << "J" << i + 1
          << " position:" << cmd[i].position << " , velocity "
          << cmd[i].velocity;
  }
  result.push_back(cmd);

  return result;
}

std::vector<std::vector<ControlCommand>>
PolynomialInterpolation::GenerateQuinticDiscreteTrajectory(
    const std::array<JointArray, 3>& init_state,
    const std::array<JointArray, 3>& end_state, double time,
    double step) const {
  int joint_num = end_state[0].size();
  std::vector<std::vector<ControlCommand>> trajectory;
  double planning_time = std::fabs(time);

  // 预计算时间幂次
  double T = planning_time, T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

  // 计算每个关节五次多项式系数
  std::vector<std::array<double, 6>> coefs(joint_num);
  for (int i = 0; i < joint_num; ++i) {
    double q0 = init_state[0][i], v0 = init_state[1][i], a0 = init_state[2][i];
    double qf = end_state[0][i], vf = end_state[1][i], af = end_state[2][i];

    auto& c = coefs[i];
    c[0] = q0;
    c[1] = v0;
    c[2] = a0 / 2.0;
    c[3] = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T2) /
           (2 * T3);
    c[4] = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T2) /
           (2 * T4);
    c[5] = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0 - af) * T2) / (2 * T5);
  }

  std::vector<ControlCommand> cmd(joint_num);
  double t = step;
  // double t = 0;
  while (t <= planning_time) {
    for (int i = 0; i < joint_num; ++i) {
      std::array<double, 6> c = coefs[i];
      double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
      // 插值位置
      cmd[i].position =
          c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
      // 速度计算
      cmd[i].velocity =
          c[1] + 2 * c[2] * t + 3 * c[3] * t2 + 4 * c[4] * t3 + 5 * c[5] * t4;
      cmd[i].acc = 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t2 + 20 * c[5] * t3;
      // AINFO << "planning_time: " << t << " ,J" << i + 1
      //       << " position:" << cmd[i].position << " , velocity "
      //       << cmd[i].velocity;
    }

    trajectory.push_back(cmd);
    t += step;
  }

  // push end state
  for (int i = 0; i < joint_num; ++i) {
    cmd[i].position = end_state[0][i];
    cmd[i].velocity = end_state[1][i];
    cmd[i].acc = end_state[2][i];
    // AINFO << "last time: "
    //       << "J" << i + 1 << " position:" << cmd[i].position << " , velocity "
    //       << cmd[i].velocity << " , acc " << cmd[i].acc;
  }
  trajectory.push_back(cmd);

  return trajectory;
}

std::vector<ControlCommand>
PolynomialInterpolation::GenerateSingleQuinticDiscreteTrajectory(
    const std::array<double, 3>& init_state,
    const std::array<double, 3>& end_state, double time, double step) const {
  std::vector<ControlCommand> trajectory;

  double planning_time = std::fabs(time);
  // 预计算时间幂次
  double T = planning_time, T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

  // 计算每个关节五次多项式系数
  std::array<double, 6> coef;
  double q0 = init_state[0], v0 = init_state[1], a0 = init_state[2];
  double qf = end_state[0], vf = end_state[1], af = end_state[2];

  coef[0] = q0;
  coef[1] = v0;
  coef[2] = a0 / 2.0;
  coef[3] =
      (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T2) / (2 * T3);
  coef[4] =
      (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T2) /
      (2 * T4);
  coef[5] =
      (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0 - af) * T2) / (2 * T5);

  ControlCommand cmd;
  double t = step;
  // double t = 0;
  while (t <= planning_time) {
    double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    // 插值位置
    cmd.position = coef[0] + coef[1] * t + coef[2] * t2 + coef[3] * t3 +
                   coef[4] * t4 + coef[5] * t5;
    // 速度计算
    cmd.velocity = coef[1] + 2 * coef[2] * t + 3 * coef[3] * t2 +
                   4 * coef[4] * t3 + 5 * coef[5] * t4;
    cmd.acc =
        2 * coef[2] + 6 * coef[3] * t + 12 * coef[4] * t2 + 20 * coef[5] * t3;
    // AINFO << "planning_time: " << t
    //       << " position:" << cmd.position << " , velocity "
    //       << cmd.velocity;

    trajectory.push_back(cmd);
    t += step;
  }

  // push end state
  cmd.position = end_state[0];
  cmd.velocity = end_state[1];
  cmd.acc = end_state[2];
  trajectory.push_back(cmd);

  return trajectory;
}

std::vector<std::vector<ControlCommand>>
PolynomialInterpolation::LinearInterpolation(const JointArray& init_state,
                                             const JointArray& end_state,
                                             double time, double step) const {
  int joint_num = end_state.size();
  double planning_time = std::fabs(time);

  std::vector<std::vector<ControlCommand>> result;
  std::vector<ControlCommand> cmd(joint_num);

  double t = 0;
  while (t <= planning_time) {
    double ratio = t / planning_time;
    for (int i = 0; i < joint_num; ++i) {
      double init_pos = init_state[i];
      double end_pos = end_state[i];

      // 线性插值公式
      cmd[i].position = init_pos + (end_pos - init_pos) * ratio;
      cmd[i].velocity = 0;  // 速度可以设为0，因为是线性插值
    }

    result.push_back(cmd);
    t += step;
  }

  // 最后一次插值设为 end_state
  for (int i = 0; i < joint_num; ++i) {
    cmd[i].position = end_state[i];
    cmd[i].velocity = 0;
  }
  result.push_back(cmd);

  return result;
}

std::vector<std::vector<ControlCommand>>
PolynomialInterpolation::TwoSegementLinearInterpolation(
    const JointArray& init_state, const JointArray& end_state, double time,
    double step) const {
  int joint_num = end_state.size();
  double planning_time = std::fabs(time);

  std::vector<std::vector<ControlCommand>> result;
  std::vector<ControlCommand> cmd(joint_num);

  double t = 0.0;
  while (t <= planning_time) {
    double ratio = t / planning_time;
    for (int i = 0; i < joint_num; ++i) {
      double init_pos = init_state[i];
      double end_pos = end_state[i];

      // 线性插值公式
      cmd[i].position = init_pos + (end_pos - init_pos) * ratio;
      cmd[i].velocity = 0;  // 速度可以设为0，因为是线性插值
    }

    result.push_back(cmd);
    t += step;
  }

  // 最后一次插值设为 end_state
  for (int i = 0; i < joint_num; ++i) {
    cmd[i].position = end_state[i];
    cmd[i].velocity = 0;
  }
  result.push_back(cmd);

  return result;
}

}  // namespace metal
}  // namespace makermods