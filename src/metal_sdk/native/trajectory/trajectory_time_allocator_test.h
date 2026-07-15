#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

struct MotionLimits {
  double vmax;  // 速度上限 |v|
  double amax;  // 加速度上限 |a|
  double jmax;  // 跃度上限 |j|
  double Tmin;  // 最小时间下限
};

struct TimeSolveResult {
  double T;
  bool feasible;
};

// 你项目里已有：using JointArray = std::array<double,6>; 或
// Eigen::Matrix<double,6,1> 等 这里仅假定 JointArray 支持 size() 与 operator[]

class TrajectoryTimeAllocator {
 public:
  // 新签名：整机（多关节）时间分配
  // init_state[0]=q0, init_state[1]=v0, init_state[2]=a0
  // end_state [0]=qf, end_state [1]=vf, end_state [2]=af
  template <typename JointArrayT>
  TimeSolveResult SolveQuinticTime(const std::array<JointArrayT, 3>& init_state,
                                   const std::array<JointArrayT, 3>& end_state,
                                   const std::vector<MotionLimits>& limits,
                                   double Tmax = 100.0) {
    const auto& q0 = init_state[0];
    const auto& v0 = init_state[1];
    const auto& a0 = init_state[2];

    const auto& qf = end_state[0];
    const auto& vf = end_state[1];
    const auto& af = end_state[2];

    const size_t dof = q0.size();
    if (limits.size() != dof) {
      return {std::numeric_limits<double>::infinity(), false};
    }

    double T_required = 0.0;
    bool all_ok = true;

    for (size_t i = 0; i < dof; ++i) {
      double Ti = solvePerJointMinTime(q0[i], v0[i], a0[i], qf[i], vf[i], af[i],
                                       limits[i], Tmax);
      if (!(Ti < Tmax)) {
        all_ok = false;
      }
      T_required = std::max(T_required, Ti);
    }

    // 再次确认：用全局 T_required 对所有关节做一遍可行性检查（保险）
    if (all_ok) {
      for (size_t i = 0; i < dof; ++i) {
        auto c = solveQuinticCoeffs(q0[i], v0[i], a0[i], qf[i], vf[i], af[i],
                                    T_required);
        if (!checkFeasibleAnalytic(c, T_required, limits[i])) {
          all_ok = false;
          break;
        }
      }
    }

    // 还要满足每个关节的 Tmin
    double Tmin_all = 0.0;
    for (const auto& lim : limits)
      Tmin_all = std::max(Tmin_all, std::max(0.01, lim.Tmin));
    T_required = std::max(T_required, Tmin_all);

    return {T_required, all_ok};
  }

 private:
  // ==== 单关节：几何扩张搜索 T（解析极值检查，无采样）====
  double solvePerJointMinTime(double q0, double v0, double a0, double qf,
                              double vf, double af, const MotionLimits& lim,
                              double Tmax) {
    double T = std::max(0.01, lim.Tmin);
    // 先快速跳到一个“肯定可行”的T
    while (T < Tmax) {
      auto c = solveQuinticCoeffs(q0, v0, a0, qf, vf, af, T);
      if (checkFeasibleAnalytic(c, T, lim)) break;
      T *= 1.5;  // 放松
    }
    if (!(T < Tmax)) return Tmax;

    // 然后在上一个不可行与当前可行之间二分收敛（解析检查，仍无采样）
    double lo = T / 1.5, hi = T;
    for (int it = 0; it < 40; ++it) {
      double mid = 0.5 * (lo + hi);
      auto c = solveQuinticCoeffs(q0, v0, a0, qf, vf, af, mid);
      if (checkFeasibleAnalytic(c, mid, lim)) {
        hi = mid;
      } else {
        lo = mid;
      }
    }
    return hi;
  }

  // ==== 五次多项式系数 ====
  std::array<double, 6> solveQuinticCoeffs(double q0, double v0, double a0,
                                           double qf, double vf, double af,
                                           double T) {
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    std::array<double, 6> c;
    c[0] = q0;
    c[1] = v0;
    c[2] = a0 / 2.0;
    c[3] = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T2) /
           (2 * T3);
    c[4] = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T2) /
           (2 * T4);
    c[5] = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0 - af) * T2) / (2 * T5);
    return c;
  }

  // ==== 解析根工具 ====
  void addIfValid(std::vector<double>& ts, double t, double T) {
    if (t > 0.0 && t < T) ts.push_back(t);
  }
  std::vector<double> solveQuadratic(double a, double b, double c) {
    std::vector<double> r;
    if (std::fabs(a) < 1e-14) {
      if (std::fabs(b) > 1e-14) r.push_back(-c / b);
      return r;
    }
    const double D = b * b - 4 * a * c;
    if (D < 0) return r;
    const double s = std::sqrt(std::max(0.0, D));
    r.push_back((-b + s) / (2 * a));
    r.push_back((-b - s) / (2 * a));
    return r;
  }
  std::vector<double> solveCubic(double a, double b, double c, double d) {
    std::vector<double> roots;
    if (std::fabs(a) < 1e-14) return solveQuadratic(b, c, d);
    // Depressed cubic: x^3 + px + q
    double A = b / a, B = c / a, C = d / a;
    double p = B - A * A / 3.0;
    double q = (2 * A * A * A) / 27.0 - (A * B) / 3.0 + C;
    double D = q * q / 4.0 + p * p * p / 27.0;
    if (D >= 0) {
      double u = std::cbrt(-q / 2.0 + std::sqrt(D));
      double v = std::cbrt(-q / 2.0 - std::sqrt(D));
      roots.push_back(u + v - A / 3.0);
    } else {
      double r = std::sqrt(-p * p * p / 27.0);
      double phi = std::acos(-q / (2 * r));
      double t = 2 * std::sqrt(-p / 3.0);
      roots.push_back(t * std::cos(phi / 3.0) - A / 3.0);
      roots.push_back(t * std::cos((phi + 2 * M_PI) / 3.0) - A / 3.0);
      roots.push_back(t * std::cos((phi + 4 * M_PI) / 3.0) - A / 3.0);
    }
    return roots;
  }

  // ==== 解析极值检查（端点 + 导数为零的点），绝对值对比上限 ====
  bool checkFeasibleAnalytic(const std::array<double, 6>& c, double T,
                             const MotionLimits& lim) {
    // q'(t) = v(t) = c1 + 2c2 t + 3c3 t^2 + 4c4 t^3 + 5c5 t^4
    // q''(t)= a(t) = 2c2 + 6c3 t + 12c4 t^2 + 20c5 t^3
    // q'''(t)=j(t) = 6c3 + 24c4 t + 60c5 t^2

    std::vector<double> cand{0.0, T};

    // 速度极值：a(t)=0 → 三次
    {
      auto vcrit = solveCubic(20 * c[5], 12 * c[4], 6 * c[3], 2 * c[2]);
      for (double t : vcrit) addIfValid(cand, t, T);
    }
    // 加速度极值：j(t)=0 → 二次
    {
      auto acrit = solveQuadratic(60 * c[5], 24 * c[4], 6 * c[3]);
      for (double t : acrit) addIfValid(cand, t, T);
    }
    // jerk 极值：j'(t)= 24c4 + 120c5 t → 线性（或端点）
    if (std::fabs(120 * c[5]) > 1e-14) {
      double t = -24 * c[4] / (120 * c[5]);
      addIfValid(cand, t, T);
    }

    // 计算这些点上的 v,a,j 并与 |.| 上限比较
    for (double t : cand) {
      double v = c[1] + 2 * c[2] * t + 3 * c[3] * t * t + 4 * c[4] * t * t * t +
                 5 * c[5] * t * t * t * t;
      double a =
          2 * c[2] + 6 * c[3] * t + 12 * c[4] * t * t + 20 * c[5] * t * t * t;
      double j = 6 * c[3] + 24 * c[4] * t + 60 * c[5] * t * t;

      if (std::fabs(v) > lim.vmax + 1e-9) return false;
      if (std::fabs(a) > lim.amax + 1e-9) return false;
      if (std::fabs(j) > lim.jmax + 1e-9) return false;
    }
    return true;
  }
};
