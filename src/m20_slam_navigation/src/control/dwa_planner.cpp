#include "m20_slam_navigation/control/dwa_planner.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace m20::control {

DWAPlanner::DWAPlanner(const LocalControllerParams& params)
    : params_(params) {}

VelocityCommand DWAPlanner::computeVelocityCommands(
    const Eigen::Matrix<Scalar, 3, 1>& current_pose,
    const VelocityCommand& current_vel,
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
    const std::vector<uint8_t>& costmap,
    int costmap_width, int costmap_height, Scalar costmap_resolution,
    Scalar costmap_origin_x, Scalar costmap_origin_y) {

  std::lock_guard<std::mutex> lock(mutex_);

  // Generate velocity samples within dynamic window
  auto samples = generateSamples(current_vel);

  VelocityCommand best_cmd = current_vel;
  DWAScore best_score;
  best_score.total = -1e9;

  // Evaluate each sample
  for (const auto& cmd : samples) {
    // Forward simulate
    auto trajectory = simulateTrajectory(current_pose, cmd,
                                          params_.sim_time, params_.sim_dt);

    if (trajectory.empty()) continue;

    // Score
    DWAScore score = scoreTrajectory(trajectory, global_path, costmap,
                                      costmap_width, costmap_height,
                                      costmap_resolution, costmap_origin_x,
                                      costmap_origin_y, current_vel);

    if (score.total > best_score.total) {
      best_score = score;
      best_cmd = cmd;
    }
  }

  return best_cmd;
}

std::vector<VelocityCommand> DWAPlanner::generateSamples(
    const VelocityCommand& current_vel) const {

  std::vector<VelocityCommand> samples;

  // Dynamic window: [v − a·dt, v + a·dt]
  Scalar vx_min = std::max(-params_.max_linear_vel_x,
                            current_vel.vx - params_.max_linear_accel_x * params_.sim_dt);
  Scalar vx_max = std::min(params_.max_linear_vel_x,
                            current_vel.vx + params_.max_linear_accel_x * params_.sim_dt);

  Scalar vy_min = std::max(-params_.max_linear_vel_y,
                            current_vel.vy - params_.max_linear_accel_y * params_.sim_dt);
  Scalar vy_max = std::min(params_.max_linear_vel_y,
                            current_vel.vy + params_.max_linear_accel_y * params_.sim_dt);

  Scalar omega_min = std::max(-params_.max_angular_vel,
                               current_vel.omega - params_.max_angular_accel * params_.sim_dt);
  Scalar omega_max = std::min(params_.max_angular_vel,
                               current_vel.omega + params_.max_angular_accel * params_.sim_dt);

  Scalar vx_step = (vx_max - vx_min) / std::max(params_.num_vx_samples - 1, 1);
  Scalar vy_step = (vy_max - vy_min) / std::max(params_.num_vy_samples - 1, 1);
  Scalar omega_step = (omega_max - omega_min) / std::max(params_.num_omega_samples - 1, 1);

  samples.reserve(params_.num_vx_samples * params_.num_vy_samples * params_.num_omega_samples);

  for (int ix = 0; ix < params_.num_vx_samples; ++ix) {
    for (int iy = 0; iy < params_.num_vy_samples; ++iy) {
      for (int io = 0; io < params_.num_omega_samples; ++io) {
        VelocityCommand cmd;
        cmd.vx    = vx_min + ix * vx_step;
        cmd.vy    = vy_min + iy * vy_step;
        cmd.omega = omega_min + io * omega_step;
        samples.push_back(cmd);
      }
    }
  }

  return samples;
}

std::vector<Eigen::Matrix<Scalar, 3, 1>> DWAPlanner::simulateTrajectory(
    const Eigen::Matrix<Scalar, 3, 1>& start_pose,
    const VelocityCommand& vel,
    Scalar sim_time, Scalar sim_dt) const {

  std::vector<Eigen::Matrix<Scalar, 3, 1>> trajectory;
  Eigen::Matrix<Scalar, 3, 1> pose = start_pose;

  int steps = static_cast<int>(sim_time / sim_dt);
  trajectory.reserve(steps + 1);
  trajectory.push_back(pose);

  for (int i = 0; i < steps; ++i) {
    // Kinematic model: holonomic
    Scalar dx = vel.vx * std::cos(pose.z()) - vel.vy * std::sin(pose.z());
    Scalar dy = vel.vx * std::sin(pose.z()) + vel.vy * std::cos(pose.z());

    pose.x() += dx * sim_dt;
    pose.y() += dy * sim_dt;
    pose.z() += vel.omega * sim_dt;
    pose.z() = math::normalize_angle(pose.z());

    trajectory.push_back(pose);
  }

  return trajectory;
}

DWAScore DWAPlanner::scoreTrajectory(
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& trajectory,
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
    const std::vector<uint8_t>& costmap,
    int costmap_width, int costmap_height, Scalar costmap_resolution,
    Scalar costmap_origin_x, Scalar costmap_origin_y,
    const VelocityCommand& prev_vel) const {

  DWAScore score;

  if (trajectory.empty() || global_path.empty()) return score;

  const Eigen::Matrix<Scalar, 3, 1>& end_pose = trajectory.back();
  const Eigen::Matrix<Scalar, 3, 1>& goal = global_path.back();

  // 1. Path clearance: min distance to obstacle along trajectory
  Scalar min_clearance = 1e9;
  for (const auto& pt : trajectory) {
    int gx = static_cast<int>(std::floor((pt.x() - costmap_origin_x) / costmap_resolution));
    int gy = static_cast<int>(std::floor((pt.y() - costmap_origin_y) / costmap_resolution));

    if (gx >= 0 && gx < costmap_width && gy >= 0 && gy < costmap_height) {
      uint8_t cost = costmap[gy * costmap_width + gx];
      if (cost >= 254) {
        score.clearance = 0;  // collision
        score.total = -1e9;
        return score;
      }
      // Higher cost → lower clearance score
      Scalar clr = 1.0 - cost / 255.0;
      if (clr < min_clearance) min_clearance = clr;
    }
  }
  score.clearance = min_clearance;

  // 2. Goal alignment: heading + distance
  Scalar dist_to_goal = std::sqrt(
      (end_pose.x() - goal.x()) * (end_pose.x() - goal.x()) +
      (end_pose.y() - goal.y()) * (end_pose.y() - goal.y()));

  Scalar heading_to_goal = std::atan2(goal.y() - end_pose.y(), goal.x() - end_pose.x());
  Scalar heading_error = std::abs(math::normalize_angle(heading_to_goal - end_pose.z()));
  Scalar heading_score = 1.0 - heading_error / math::kPI;
  Scalar dist_score = std::max(0.0, 1.0 - dist_to_goal / 3.0);  // normalize to 3m
  score.goal_align = 0.5 * heading_score + 0.5 * dist_score;

  // 3. Velocity progress: how far along the global path
  // Find closest point on global path
  Scalar min_path_dist = 1e9;
  int closest_idx = 0;
  for (int i = 0; i < static_cast<int>(global_path.size()); ++i) {
    Scalar d = std::sqrt(
        (end_pose.x() - global_path[i].x()) * (end_pose.x() - global_path[i].x()) +
        (end_pose.y() - global_path[i].y()) * (end_pose.y() - global_path[i].y()));
    if (d < min_path_dist) {
      min_path_dist = d;
      closest_idx = i;
    }
  }
  // Progress = fraction along the path
  score.progress = static_cast<Scalar>(closest_idx) /
                   std::max(static_cast<Scalar>(global_path.size() - 1), Scalar(1));

  // 4. Terrain cost: average costmap value along trajectory
  Scalar terrain_sum = 0;
  int terrain_count = 0;
  for (const auto& pt : trajectory) {
    int gx = static_cast<int>(std::floor((pt.x() - costmap_origin_x) / costmap_resolution));
    int gy = static_cast<int>(std::floor((pt.y() - costmap_origin_y) / costmap_resolution));
    if (gx >= 0 && gx < costmap_width && gy >= 0 && gy < costmap_height) {
      terrain_sum += costmap[gy * costmap_width + gx] / 255.0;
      terrain_count++;
    }
  }
  score.terrain = 1.0 - (terrain_count > 0 ? terrain_sum / terrain_count : 1.0);

  // 5. Smoothness: penalize large changes from previous velocity
  Scalar vel_diff = std::abs(prev_vel.vx - trajectory.size() > 0 ? 0 : 0);  // simplified
  score.smoothness = 0.7;  // placeholder

  // Weighted total
  score.total = params_.weight_path_clearance * score.clearance
              + params_.weight_goal_align * score.goal_align
              + params_.weight_velocity_progress * score.progress
              + params_.weight_terrain_cost * score.terrain
              + params_.weight_smoothness * score.smoothness;

  return score;
}

}  // namespace m20::control