#pragma once
/**
 * @file local_controller_node.hpp
 * @brief Local controller orchestrator: DWA ↔ LinePlanner mode switching.
 *
 * On each control cycle:
 *   1. Check if global path ahead is straight → LinePlanner mode.
 *   2. Otherwise → DWA mode.
 *   3. Publish /cmd_vel as geometry_msgs/Twist.
 *
 * Frequency: 50Hz (aligned with control loop, not LiDAR rate).
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/control/dwa_planner.hpp"
#include "m20_slam_navigation/control/line_planner.hpp"

#include <Eigen/Dense>

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace m20::control {

using VelocityCallback = std::function<void(const VelocityCommand& cmd)>;

class LocalControllerNode {
public:
  LocalControllerNode(const LocalControllerParams& controller_params);

  /**
   * @brief Compute and publish velocity command.
   *
   * @param current_pose      Robot pose [x, y, θ]
   * @param current_vel       Current velocity (vx, vy, omega)
   * @param global_path       Global plan waypoints
   * @param costmap           Traversability costmap
   * @param costmap_width     Width [cells]
   * @param costmap_height    Height [cells]
   * @param costmap_resolution Resolution [m/cell]
   * @param costmap_origin_x  Origin x
   * @param costmap_origin_y  Origin y
   * @return                  Velocity command
   */
  VelocityCommand step(
      const Eigen::Matrix<Scalar, 3, 1>& current_pose,
      const VelocityCommand& current_vel,
      const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
      const std::vector<uint8_t>& costmap,
      int costmap_width, int costmap_height, Scalar costmap_resolution,
      Scalar costmap_origin_x, Scalar costmap_origin_y);

  /// Set velocity callback
  void setVelocityCallback(VelocityCallback cb) { vel_cb_ = std::move(cb); }

  /// Get current planner mode
  enum class Mode { LINE, DWA };
  Mode getMode() const { return current_mode_; }

  /// Set parameters
  void setParams(const LocalControllerParams& params);

  /// Native /planner_mode control: 0=automatic, 1=LinePlanner, 2=DWA.
  void setPlannerMode(int mode);

  /// Cancel the active navigation request and force a zero command.
  void cancel();

  /// Clear a previous cancellation.
  void resume();

private:
  LocalControllerParams                params_;
  std::unique_ptr<DWAPlanner>          dwa_planner_;
  std::unique_ptr<LinePlanner>         line_planner_;
  Mode                                 current_mode_{Mode::DWA};
  int                                  requested_mode_{0};
  bool                                 canceled_{false};
  VelocityCallback                     vel_cb_;
  mutable std::mutex                   mutex_;
};

}  // namespace m20::control
