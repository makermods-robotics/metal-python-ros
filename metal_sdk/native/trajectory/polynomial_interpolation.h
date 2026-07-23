#pragma once

#include <array>
#include <vector>

#include "common/motor_state.h"

namespace makermods {
namespace metal {

using JointArray = std::vector<double>;

class PolynomialInterpolation {
 public:
  PolynomialInterpolation() = default;
  ~PolynomialInterpolation() = default;

 public:
  /**
   * @brief Generate cubic discrete trajectory points
   * @param init_state: initial state, 0 position vector, 1 velocity vector
   * @param end_state: end state, 0 position vector, 1 velocity vector
   * @param time: total time
   * @param step: discrete time step
   * @return: discrete trajectory points, include position and velocity
   */
  std::vector<std::vector<ControlCommand>> GenerateCubicDiscreteTrajectory(
      std::pair<JointArray, JointArray> init_state,
      std::pair<JointArray, JointArray> end_state, double time,
      double step) const;

  /**
   * @brief Generate quintic discrete trajectory points
   * @param init_state: initial state, {pos, vel, accel}
   * @param end_state: end state, {pos, vel, accel}
   * @param time: total time
   * @param step: discrete time step
   * @return: discrete trajectory points, include position and velocity
   */
  std::vector<std::vector<ControlCommand>> GenerateQuinticDiscreteTrajectory(
      const std::array<JointArray, 3>& init_state,
      const std::array<JointArray, 3>& end_state, double time,
      double step) const;

  /**
   * @brief Generate single joint quintic discrete trajectory points
   * @param init_state: initial state, {pos, vel, accel}
   * @param end_state: end state, {pos, vel, accel}
   * @param time: total time
   * @param step: discrete time step
   * @return: single joint discrete trajectory points, include position and
   * velocity
   */
  std::vector<ControlCommand> GenerateSingleQuinticDiscreteTrajectory(
      const std::array<double, 3>& init_state,
      const std::array<double, 3>& end_state, double time, double step) const;

  std::vector<std::vector<ControlCommand>> LinearInterpolation(
      const JointArray& init_state, const JointArray& end_state, double time,
      double step) const;

  std::vector<std::vector<ControlCommand>> TwoSegementLinearInterpolation(
      const JointArray& init_state, const JointArray& end_state, double time,
      double step) const;
};

}  // namespace metal
}  // namespace makermods
