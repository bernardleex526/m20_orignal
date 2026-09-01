#include "m20_slam_navigation/control/local_controller_node.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace m20::control {

LocalControllerNode::LocalControllerNode(const LocalControllerParams& controller_params)
    : params_(controller_params)
    , dwa_planner_(std::make_unique<DWAPlanner>(controller_params))
    , line_planner_(std::make_unique<LinePlanner>(controller_params)) {
}

VelocityCommand LocalControllerNode::step(
    const Eigen::Matrix<Scalar, 3, 1>& current_pose,
    const VelocityCommand& current_vel,
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
    const std::vector<uint8_t>& costmap,
    int costmap_width, int costmap_height, Scalar costmap_resolution,
    Scalar costmap_origin_x, Scalar costmap_origin_y) {

  std::lock_guard<std::mutex> lock(mutex_);

  VelocityCommand cmd;

  if (canceled_ || global_path.empty()) {
    current_mode_ = Mode::DWA;
    if (vel_cb_) vel_cb_(cmd);
    return cmd;
  }

  const auto& final_goal = global_path.back();
  const Scalar goal_distance = std::hypot(
      current_pose.x() - final_goal.x(), current_pose.y() - final_goal.y());
  if (goal_distance <= params_.stop_distance) {
    const Scalar yaw_error = math::normalize_angle(final_goal.z() - current_pose.z());
    if (std::abs(yaw_error) > params_.yaw_tolerance) {
      current_mode_ = Mode::DWA;
      cmd.omega = std::clamp(
          params_.turn_yaw_kp * yaw_error,
          -params_.native_max_theta, params_.native_max_theta);
    } else {
      current_mode_ = Mode::DWA;
    }
    if (vel_cb_) vel_cb_(cmd);
    return cmd;
  }

  const bool force_line = requested_mode_ == 1;
  const bool force_dwa = requested_mode_ == 2;
  const bool line_mode = !force_dwa &&
      (force_line || (params_.direct_line_mode &&
                      line_planner_->isLineMode(
                          global_path, current_pose, params_.look_ahead_dis)));

  // Check if LinePlanner mode applies
  if (line_mode) {
    current_mode_ = Mode::LINE;

    // Extract the line segment ahead
    if (global_path.size() >= 2) {
      Eigen::Matrix<Scalar, 2, 1> line_start(global_path.front().x(), global_path.front().y());
      Eigen::Matrix<Scalar, 2, 1> line_end(global_path.back().x(), global_path.back().y());

      // Adjust line_start to closest point on path
      Scalar min_dist = 1e9;
      for (const auto& wp : global_path) {
        Scalar d = std::sqrt(
            (current_pose.x() - wp.x()) * (current_pose.x() - wp.x()) +
            (current_pose.y() - wp.y()) * (current_pose.y() - wp.y()));
        if (d < min_dist) {
          min_dist = d;
          line_start = Eigen::Matrix<Scalar, 2, 1>(wp.x(), wp.y());
        }
      }

      cmd = line_planner_->computeLineCommand(
          current_pose, line_start, line_end, params_.native_max_speed_x);
    } else {
      // Fall through to DWA
      current_mode_ = Mode::DWA;
      cmd = dwa_planner_->computeVelocityCommands(
          current_pose, current_vel, global_path,
          costmap, costmap_width, costmap_height, costmap_resolution,
          costmap_origin_x, costmap_origin_y);
    }
  } else {
    current_mode_ = Mode::DWA;
    cmd = dwa_planner_->computeVelocityCommands(
        current_pose, current_vel, global_path,
        costmap, costmap_width, costmap_height, costmap_resolution,
        costmap_origin_x, costmap_origin_y);
  }

  if (vel_cb_) {
    vel_cb_(cmd);
  }

  return cmd;
}

void LocalControllerNode::setParams(const LocalControllerParams& params) {
  std::lock_guard<std::mutex> lock(mutex_);
  params_ = params;
  dwa_planner_->setParams(params);
  line_planner_->setParams(params);
}

void LocalControllerNode::setPlannerMode(int mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  requested_mode_ = mode;
}

void LocalControllerNode::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  canceled_ = true;
}

void LocalControllerNode::resume() {
  std::lock_guard<std::mutex> lock(mutex_);
  canceled_ = false;
}

}  // namespace m20::control
