#include "m20_slam_navigation/control/local_controller_node.hpp"

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

  // Check if LinePlanner mode applies
  if (line_planner_->isLineMode(global_path, current_pose)) {
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
          current_pose, line_start, line_end, params_.max_linear_vel_x);
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
}

}  // namespace m20::control