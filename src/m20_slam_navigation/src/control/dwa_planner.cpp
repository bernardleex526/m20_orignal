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

  // If every candidate collides, stop rather than replaying a stale or
  // out-of-range measured velocity.
  VelocityCommand best_cmd;
  DWAScore best_score;
  best_score.total = -1e9;
  bool found_valid_command = false;

  // Evaluate each sample
  for (const auto& cmd : samples) {
    // Forward simulate
    auto trajectory = simulateTrajectory(current_pose, cmd,
                                          params_.sim_time, params_.sim_dt);

    if (trajectory.empty()) continue;

    // Score
    DWAScore score = scoreTrajectory(trajectory, current_pose, global_path, costmap,
                                      costmap_width, costmap_height,
                                      costmap_resolution, costmap_origin_x,
                                      costmap_origin_y, current_vel);

    if (score.total > best_score.total) {
      best_score = score;
      best_cmd = cmd;
      found_valid_command = true;
    }
  }

  if (!found_valid_command) return VelocityCommand{};
  return best_cmd;
}

std::vector<VelocityCommand> DWAPlanner::generateSamples(
    const VelocityCommand& current_vel) const {

  std::vector<VelocityCommand> samples;

  // The native localPlanner limits the command with maxSpeedX/Y/maxTheta and
  // applies an acceleration threshold at the control period.  Keep the
  // generic fields as a backwards-compatible fallback for old YAML files.
  const Scalar max_vx = params_.native_max_speed_x > 0.0
      ? params_.native_max_speed_x : params_.max_linear_vel_x;
  const Scalar max_vy = params_.native_max_speed_y > 0.0
      ? params_.native_max_speed_y : params_.max_linear_vel_y;
  const Scalar max_omega = params_.native_max_theta > 0.0
      ? params_.native_max_theta : params_.max_angular_vel;
  const Scalar accel_x = params_.velocity_accel_threshold > 0.0
      ? params_.velocity_accel_threshold : params_.max_linear_accel_x;
  const Scalar accel_y = params_.velocity_accel_threshold > 0.0
      ? params_.velocity_accel_threshold : params_.max_linear_accel_y;
  const Scalar accel_omega = params_.velocity_accel_threshold > 0.0
      ? params_.velocity_accel_threshold : params_.max_angular_accel;

  // Dynamic window: [v − a·dt, v + a·dt].  Native backward_mode=false
  // rejects reverse samples; enable them only when explicitly requested.
  const Scalar vx_floor = params_.backward_mode ? -max_vx : 0.0;
  const Scalar center_vx = std::clamp(current_vel.vx, vx_floor, max_vx);
  const Scalar center_vy = std::clamp(current_vel.vy, -max_vy, max_vy);
  const Scalar center_omega = std::clamp(current_vel.omega, -max_omega, max_omega);
  Scalar vx_min = std::max(vx_floor, center_vx - accel_x * params_.sim_dt);
  Scalar vx_max = std::min(max_vx, center_vx + accel_x * params_.sim_dt);

  Scalar vy_min = std::max(-max_vy, center_vy - accel_y * params_.sim_dt);
  Scalar vy_max = std::min(max_vy, center_vy + accel_y * params_.sim_dt);

  Scalar omega_min = std::max(-max_omega, center_omega - accel_omega * params_.sim_dt);
  Scalar omega_max = std::min(max_omega, center_omega + accel_omega * params_.sim_dt);

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
    const Eigen::Matrix<Scalar, 3, 1>& current_pose,
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
    const std::vector<uint8_t>& costmap,
    int costmap_width, int costmap_height, Scalar costmap_resolution,
    Scalar costmap_origin_x, Scalar costmap_origin_y,
    const VelocityCommand& prev_vel) const {

  DWAScore score;

  if (trajectory.empty() || global_path.empty()) return score;
  if (costmap_width <= 0 || costmap_height <= 0 || costmap_resolution <= 0.0 ||
      costmap.size() < static_cast<std::size_t>(costmap_width) *
                           static_cast<std::size_t>(costmap_height)) {
    score.total = -1e9;
    return score;
  }

  const Eigen::Matrix<Scalar, 3, 1>& end_pose = trajectory.back();
  const Eigen::Matrix<Scalar, 3, 1>& goal = global_path.back();

  // 1. Path clearance: inspect the native rectangular M20 footprint rather
  // than only the trajectory centre point.
  Scalar min_clearance = 1.0;
  const Scalar half_length = std::max(params_.robot_length * 0.5, Scalar(0.01));
  const Scalar half_width = std::max(params_.robot_width * 0.5, Scalar(0.01));
  for (const auto& pt : trajectory) {
    const Scalar c = std::cos(pt.z());
    const Scalar s = std::sin(pt.z());
    const int length_steps = std::max(
        1, static_cast<int>(std::ceil(half_length / costmap_resolution)));
    const int width_steps = std::max(
        1, static_cast<int>(std::ceil(half_width / costmap_resolution)));
    for (int il = -length_steps; il <= length_steps; ++il) {
      for (int iw = -width_steps; iw <= width_steps; ++iw) {
        const Scalar local_x = il * costmap_resolution;
        const Scalar local_y = iw * costmap_resolution;
        const Scalar x = pt.x() + c * local_x - s * local_y;
        const Scalar y = pt.y() + s * local_x + c * local_y;
        int gx = static_cast<int>(std::floor((x - costmap_origin_x) / costmap_resolution));
        int gy = static_cast<int>(std::floor((y - costmap_origin_y) / costmap_resolution));

        if (gx < 0 || gx >= costmap_width || gy < 0 || gy >= costmap_height) {
          score.clearance = 0;
          score.total = -1e9;
          return score;
        }
        uint8_t cost = costmap[gy * costmap_width + gx];
        if (cost >= 254) {
          score.clearance = 0;  // collision
          score.total = -1e9;
          return score;
        }
        // Preserve a continuous obstacle score for the native ob1/ob2/ob3
        // terms while treating 254/255 as a hard collision above.
        min_clearance = std::min(min_clearance, 1.0 - cost / 255.0);
      }
    }
  }
  score.clearance = min_clearance;

  // 2. Goal and yaw terms.  Native localPlanner separately weights goal yaw,
  // so use the final pose orientation when it is available.
  Scalar dist_to_goal = std::sqrt(
      (end_pose.x() - goal.x()) * (end_pose.x() - goal.x()) +
      (end_pose.y() - goal.y()) * (end_pose.y() - goal.y()));

  Scalar heading_to_goal = goal.z();
  if (dist_to_goal > 1e-3) {
    heading_to_goal = std::atan2(goal.y() - end_pose.y(), goal.x() - end_pose.x());
  }
  Scalar heading_error = std::abs(math::normalize_angle(heading_to_goal - end_pose.z()));
  Scalar heading_score = 1.0 - std::min(heading_error / math::kPI, Scalar(1.0));
  const Scalar goal_range = std::max(params_.dwa_fine_tune_distance, Scalar(0.5));
  Scalar dist_score = std::max(0.0, 1.0 - dist_to_goal / goal_range);
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

  // 5. Smoothness: compare the sampled command against the measured command.
  // The old implementation contained a precedence bug and always returned a
  // constant.  This term is now deterministic and bounded in [0, 1].
  const auto& first = trajectory.size() > 1 ? trajectory[1] : trajectory.front();
  const Scalar dt = std::max(params_.sim_dt, Scalar(1e-3));
  const Scalar est_vx = (first.x() - current_pose.x()) * std::cos(current_pose.z()) / dt +
                        (first.y() - current_pose.y()) * std::sin(current_pose.z()) / dt;
  const Scalar est_vy = -(first.x() - current_pose.x()) * std::sin(current_pose.z()) / dt +
                        (first.y() - current_pose.y()) * std::cos(current_pose.z()) / dt;
  const Scalar vel_diff = std::hypot(est_vx - prev_vel.vx, est_vy - prev_vel.vy) +
                          std::abs((first.z() - current_pose.z()) / dt - prev_vel.omega);
  const Scalar vel_scale = std::max(
      Scalar(1.0), params_.native_max_speed_x + params_.native_max_theta);
  score.smoothness = std::max(0.0, 1.0 - vel_diff / vel_scale);

  // Native localplanner objective.  The generic weights remain accepted for
  // old configurations, but the shipped native YAML maps to these terms.
  score.total = params_.weight_ob1 * score.clearance
              + params_.weight_goal * dist_score
              + params_.weight_yaw * heading_score
              + params_.weight_spdy * score.progress
              + params_.weight_ob2 * score.terrain
              + params_.weight_ob3 * score.goal_align
              + params_.weight_smoothness * score.smoothness;

  return score;
}

}  // namespace m20::control
