#pragma once
#include <algorithm>
#include <cmath>

#include "common/log.h"

namespace makermods {
namespace metal {

struct MotionLimits {
  double vmax;  // 允许峰值速度 [rad/s]
  double amax;  // 允许峰值加速度 [rad/s^2]
  double jmax;  // 允许峰值跃度 [rad/s^3]
  double Tmin;  // 可选的全局最小时间 [s]
};

struct TimeSolveResult {
  double T;  // 分配后的时间
  double Tv, Ta, Tj, Tmin;
};

class TrajectoryTimeAllocator {
 public:
  // speed_factor: 外置“速度趋势”系数，1.0为基准。建议 >= 0.1
  TimeSolveResult SolveQuinticTime(double dtheta, double speed_ratio,
                                   bool is_traj, bool is_gripper = false) {
    const double adtheta = std::fabs(dtheta);

    // 归一化峰值常数（标准 quintic）
    constexpr double K_v = 1.875;   // vmax_norm
    constexpr double K_a = 5.7735;  // amax_norm
    constexpr double K_j = 60.0;    // jmax_norm

    // 依据外置趋势缩放上限（方式A：同比例缩放）
    MotionLimits lim = point_control_jointLimits;
    double Tmin = point_control_jointLimits.Tmin;

    if (is_gripper) {
      if (is_traj) {
        lim = traj_gripper_limit;
        Tmin = traj_gripper_limit.Tmin;
        // AINFO << "gripper trajectory";
      } else {
        lim = point_gripper_limit;
        Tmin = point_gripper_limit.Tmin;
      }
    } else {
      if (is_traj) {
        lim = traj_control_jointLimits;
        Tmin = traj_control_jointLimits.Tmin;
        // AINFO << "arm trajectory";
      }
    }
    // AINFO << "speed_ratio: " << speed_ratio;
    lim.vmax = std::fmax(1e-6, speed_ratio * lim.vmax);
    lim.amax = std::fmax(1e-6, speed_ratio * lim.amax);
    lim.jmax = std::fmax(1e-6, speed_ratio * lim.jmax);

    // 各约束对应的最小时间
    double Tv = (adtheta <= 0.0) ? 0.0 : (K_v * adtheta / lim.vmax);
    double Ta = (adtheta <= 0.0) ? 0.0 : std::sqrt(K_a * adtheta / lim.amax);
    double Tj = (adtheta <= 0.0) ? 0.0 : std::cbrt(K_j * adtheta / lim.jmax);

    // AINFO << "Tv: " << Tv;
    // AINFO << "Ta: " << Ta;
    // AINFO << "Tj: " << Tj;

    double T = std::max({Tv, Ta, Tmin});
    // 避免极小 T
    if (T < 1e-4) T = 1e-4;

    return {T, Tv, Ta, Tj, Tmin};
  }

 private:
  MotionLimits point_control_jointLimits = {3.14, 13.0, 20.0, 0.5};
  MotionLimits traj_control_jointLimits = {10.0, 3000.0, 500.0, 0.0};
  MotionLimits point_gripper_limit = {3, 5, 20, 0.2};
  MotionLimits traj_gripper_limit = {5, 100, 500, 0.0};
};

}  // namespace metal
}  // namespace makermods