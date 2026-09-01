#include "m20_slam_navigation/control/line_planner.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace m20::control {

LinePlanner::LinePlanner(const LocalControllerParams& params)
    : params_(params) {}

bool LinePlanner::isLineMode(
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& global_path,
    const Eigen::Matrix<Scalar, 3, 1>& robot_pose,
    Scalar lookahead_distance) const {

  if (global_path.size() < 2) return false;

  lookahead_distance = lookahead_distance > 0.0
      ? lookahead_distance : params_.look_ahead_dis;

  // Check if path ahead is approximately straight
  Scalar dist_accum = 0;
  Scalar prev_theta = global_path[0].z();
  Scalar total_turn = 0;

  for (std::size_t i = 1; i < global_path.size() && dist_accum < lookahead_distance; ++i) {
    Scalar theta = global_path[i].z();
    Scalar dtheta = std::abs(math::normalize_angle(theta - prev_theta));

    const Scalar segment_length = std::hypot(
        global_path[i].x() - global_path[i-1].x(),
        global_path[i].y() - global_path[i-1].y());
    if (segment_length < params_.min_segment_length) {
      prev_theta = theta;
      continue;
    }
    total_turn += dtheta;
    if (dtheta > params_.angle_threshold_deg * math::kPI / 180.0 ||
        total_turn > params_.sum_angle_threshold_deg * math::kPI / 180.0 ||
        dtheta > params_.lineplanner_yaw_threshold) {
      return false;
    }

    dist_accum += segment_length;
    prev_theta = theta;
  }

  // Native directLine mode is only selected when the robot is close enough to
  // the path laterally.  This avoids switching to LinePlanner for a distant
  // waypoint sequence that merely happens to be straight.
  const auto& first = global_path.front();
  const Scalar distance_to_first = std::hypot(
      robot_pose.x() - first.x(), robot_pose.y() - first.y());
  return distance_to_first <= std::max(params_.track_distance, lookahead_distance);
}

VelocityCommand LinePlanner::computeLineCommand(
    const Eigen::Matrix<Scalar, 3, 1>& robot_pose,
    const Eigen::Matrix<Scalar, 2, 1>& line_start,
    const Eigen::Matrix<Scalar, 2, 1>& line_end,
    Scalar target_speed) const {

  VelocityCommand cmd;

  // Line direction
  Eigen::Matrix<Scalar, 2, 1> line_dir = line_end - line_start;
  Scalar line_length = line_dir.norm();
  if (line_length < 1e-6) {
    cmd.vx = 0; cmd.vy = 0; cmd.omega = 0;
    return cmd;
  }
  line_dir /= line_length;

  // Robot position
  Eigen::Matrix<Scalar, 2, 1> robot_xy(robot_pose.x(), robot_pose.y());

  // Cross-track error: signed distance from robot to line
  Scalar cross_track = crossTrackError(robot_xy, line_start, line_end);

  // Forward velocity: proportional to distance to end, capped at target_speed
  Scalar dist_to_end = (line_end - robot_xy).norm();
  const Scalar max_speed = params_.native_max_speed_x > 0.0
      ? params_.native_max_speed_x : params_.max_linear_vel_x;
  Scalar speed_scale = 1.0;
  if (params_.stop_distance > 0.0 && dist_to_end < params_.stop_distance) {
    speed_scale = std::max(0.0, dist_to_end / params_.stop_distance);
  }
  Scalar vx = std::min(target_speed, max_speed) * speed_scale;
  cmd.vx = math::clamp(vx, params_.backward_mode ? -max_speed : Scalar(0), max_speed);

  // Lateral velocity: P-controller for cross-track error
  // v_y = −K_p · e_y  (negative to move toward the line)
  const Scalar max_lateral = params_.native_max_speed_y > 0.0
      ? params_.native_max_speed_y : params_.max_linear_vel_y;
  cmd.vy = math::clamp(-params_.turn_yaw_kp * cross_track,
                        -max_lateral, max_lateral);

  // Heading correction: align with line direction
  Scalar line_heading = std::atan2(line_dir.y(), line_dir.x());
  Scalar heading_error = math::normalize_angle(line_heading - robot_pose.z());

  // Small P-control on heading
  const Scalar max_yaw = params_.native_max_theta > 0.0
      ? params_.native_max_theta : params_.max_angular_vel;
  cmd.omega = math::clamp(heading_error * params_.turn_yaw_kp,
                           -max_yaw, max_yaw);

  return cmd;
}

Scalar LinePlanner::crossTrackError(
    const Eigen::Matrix<Scalar, 2, 1>& point,
    const Eigen::Matrix<Scalar, 2, 1>& line_start,
    const Eigen::Matrix<Scalar, 2, 1>& line_end) {

  // Signed distance from point to line segment
  Eigen::Matrix<Scalar, 2, 1> line_vec = line_end - line_start;
  Scalar line_len = line_vec.norm();
  if (line_len < 1e-10) {
    return (point - line_start).norm();
  }

  Eigen::Matrix<Scalar, 2, 1> line_dir = line_vec / line_len;
  Eigen::Matrix<Scalar, 2, 1> to_point = point - line_start;

  // Project onto line
  Scalar t = to_point.dot(line_dir);
  t = math::clamp(t, Scalar(0), line_len);

  // Closest point on segment
  Eigen::Matrix<Scalar, 2, 1> closest = line_start + t * line_dir;

  // Signed distance: cross product of line_dir with (point - line_start)
  // Positive when point is to the LEFT of the line (in 2D)
  Scalar signed_dist = line_dir.x() * (point.y() - closest.y()) -
                       line_dir.y() * (point.x() - closest.x());

  return signed_dist;
}

}  // namespace m20::control
