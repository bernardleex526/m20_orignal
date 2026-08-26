#pragma once
/**
 * @file line_planner.hpp
 * @brief Fast straight-line tracking controller for narrow corridors.
 *
 * When the global path ahead is approximately straight (no significant yaw
 * change within the lookahead distance), this controller switches to a
 * simpler and more robust mode:
 *
 *   - Forward velocity v_x is set based on distance to goal/waypoint.
 *   - Lateral velocity v_y compensates for cross-track error without yaw
 *     rotation (exploiting quadruped omnidirectionality).
 *   - Angular velocity ω_z only corrects heading when deviation exceeds
 *     a threshold.
 *
 * Control law (decoupled lateral):
 *   v_y = −K_p · e_y − K_d · ė_y
 *
 * where e_y is the cross-track error (lateral distance to the line) and
 * ė_y = v · sin(heading_error).
 *
 * This mode is especially effective in long corridors where the robot
 * should move straight with active lateral drift compensation, avoiding
 * unnecessary yaw oscillations from DWA sampling.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Dense>

#include <vector>

namespace m20::control {

class LinePlanner {
public:
  LinePlanner(const LocalControllerParams& params);

  /**
   * @brief Check if the path ahead qualifies for LinePlanner mode.
   *
   * Returns true if the path ahead (within lookahead distance) has
   * heading changes below yaw_threshold.
   */
  bool isLineMode(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
                  const Eigen::Matrix<Scalar, 3, 1>& robot_pose,
                  Scalar lookahead_distance = 3.0) const;

  /**
   * @brief Compute velocity command for line-following mode.
   *
   * @param robot_pose    Current robot [x, y, θ]
   * @param line_start    Start point of the line segment [x, y]
   * @param line_end      End point of the line segment [x, y]
   * @param target_speed  Desired forward speed [m/s]
   * @return              Velocity command
   */
  VelocityCommand computeLineCommand(
      const Eigen::Matrix<Scalar, 3, 1>& robot_pose,
      const Eigen::Matrix<Scalar, 2, 1>& line_start,
      const Eigen::Matrix<Scalar, 2, 1>& line_end,
      Scalar target_speed) const;

private:
  /// Compute cross-track error: signed distance from point to line
  static Scalar crossTrackError(const Eigen::Matrix<Scalar, 2, 1>& point,
                                const Eigen::Matrix<Scalar, 2, 1>& line_start,
                                const Eigen::Matrix<Scalar, 2, 1>& line_end);

  LocalControllerParams params_;
};

}  // namespace m20::control