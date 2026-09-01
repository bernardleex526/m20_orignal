#pragma once
/**
 * @file dwa_planner.hpp
 * @brief Omnidirectional Dynamic Window Approach (DWA) local planner.
 *
 * DWA samples the velocity space (v_x, v_y, ω_z) within the dynamic window
 * defined by current velocity and acceleration limits, then forward-simulates
 * each candidate trajectory for a short horizon, scoring them on:
 *
 *   1. Path clearance: distance to nearest obstacle (costmap).
 *   2. Goal alignment: heading and distance to global plan waypoint.
 *   3. Velocity progress: how far along the global path the robot moves.
 *   4. Terrain traversability cost: integrated terrain cost along trajectory.
 *   5. Smoothness: penalty for velocity changes from previous command.
 *
 * Total score:
 *   S = w_clear·c_clear + w_goal·c_goal + w_progress·c_progress
 *     + w_terrain·c_terrain + w_smooth·c_smooth
 *
 * The omnidirectional variant samples all 3 DOFs independently, enabling
 * lateral (sideways) motion without yaw rotation — critical for narrow
 * corridors where quadruped can slide sideways.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Dense>

#include <memory>
#include <mutex>
#include <vector>

namespace m20::control {

struct DWAScore {
  Scalar clearance{0};      ///< obstacle distance score [0, 1] (1 = clear)
  Scalar goal_align{0};     ///< goal alignment score [0, 1] (1 = perfectly aligned)
  Scalar progress{0};       ///< velocity progress score [0, 1]
  Scalar terrain{0};        ///< terrain traversability score [0, 1]
  Scalar smoothness{0};     ///< velocity continuity score [0, 1]
  Scalar total{0};          ///< weighted sum
};

struct VelocityCommand {
  Scalar vx{0};    ///< forward velocity [m/s]
  Scalar vy{0};    ///< lateral velocity [m/s]
  Scalar omega{0}; ///< angular velocity [rad/s]
};

class DWAPlanner {
public:
  DWAPlanner(const LocalControllerParams& params);

  /**
   * @brief Compute best velocity command.
   *
   * @param current_pose       Robot pose [x, y, θ]
   * @param current_vel        Current velocity (vx, vy, omega)
   * @param global_path        Global plan waypoints
   * @param costmap            Traversability costmap [0, 255]
   * @param costmap_width      Costmap width
   * @param costmap_height     Costmap height
   * @param costmap_resolution Costmap resolution
   * @param costmap_origin_x   Costmap origin x
   * @param costmap_origin_y   Costmap origin y
   * @return                   Best velocity command
   */
  VelocityCommand computeVelocityCommands(
      const Eigen::Matrix<Scalar, 3, 1>& current_pose,
      const VelocityCommand& current_vel,
      const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
      const std::vector<uint8_t>& costmap,
      int costmap_width, int costmap_height, Scalar costmap_resolution,
      Scalar costmap_origin_x, Scalar costmap_origin_y);

  /// Set parameters dynamically
  void setParams(const LocalControllerParams& params) { params_ = params; }

private:
  /// Generate velocity samples within dynamic window
  std::vector<VelocityCommand> generateSamples(const VelocityCommand& current_vel) const;

  /// Forward-simulate trajectory for a velocity sample
  std::vector<Eigen::Matrix<Scalar, 3, 1>> simulateTrajectory(
      const Eigen::Matrix<Scalar, 3, 1>& start_pose,
      const VelocityCommand& vel,
      Scalar sim_time, Scalar sim_dt) const;

  /// Score a trajectory
  DWAScore scoreTrajectory(
      const std::vector<Eigen::Matrix<Scalar, 3, 1>>& trajectory,
      const Eigen::Matrix<Scalar, 3, 1>& current_pose,
      const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
      const std::vector<uint8_t>& costmap,
      int costmap_width, int costmap_height, Scalar costmap_resolution,
      Scalar costmap_origin_x, Scalar costmap_origin_y,
      const VelocityCommand& prev_vel) const;

  LocalControllerParams params_;
  mutable std::mutex    mutex_;
};

}  // namespace m20::control
