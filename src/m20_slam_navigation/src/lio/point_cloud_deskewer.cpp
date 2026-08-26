#include "m20_slam_navigation/lio/point_cloud_deskewer.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <chrono>

namespace m20::lio {

PointCloudDeskewer::PointCloudDeskewer(const SE3Pose& T_lidar_imu)
    : T_lidar_imu_(T_lidar_imu) {}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudDeskewer::deskew(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_cloud,
    const std::vector<std::pair<Timestamp, SE3Pose>>& trajectory,
    const Timestamp& scan_start) {

  auto deskewed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  deskewed->reserve(raw_cloud->size());
  deskewed->header = raw_cloud->header;

  if (trajectory.empty()) {
    // No trajectory: return as-is
    *deskewed = *raw_cloud;
    return deskewed;
  }

  // Get scan end time from trajectory
  Timestamp scan_end = trajectory.back().first;

  for (const auto& point : raw_cloud->points) {
    // Per-point timestamp: stored in intensity field as time offset [s] from scan start
    Scalar time_offset_s = point.intensity;
    auto pt_stamp = scan_start + std::chrono::nanoseconds(
        static_cast<int64_t>(time_offset_s * 1e9));

    // Clamp to scan interval
    if (pt_stamp < scan_start) pt_stamp = scan_start;
    if (pt_stamp > scan_end)  pt_stamp = scan_end;

    // Interpolate pose at point timestamp
    SE3Pose T_i = interpolatePose(trajectory, pt_stamp);

    // Relative motion from point time to scan start: ΔT = T_start⁻¹ ∘ T_i
    const SE3Pose& T_start = trajectory.front().second;
    SE3Pose delta_T = T_start.inverse() * T_i;

    // Transform point: p_corrected = T_lidar_imu⁻¹ · ΔT · T_lidar_imu · p_raw
    Eigen::Matrix<Scalar, 3, 1> p_raw(point.x, point.y, point.z);

    // Step 1: LiDAR → IMU frame
    Eigen::Matrix<Scalar, 3, 1> p_imu = T_lidar_imu_.transformPoint(p_raw);

    // Step 2: Apply relative motion ΔT
    Eigen::Matrix<Scalar, 3, 1> p_corrected_imu = delta_T.transformPoint(p_imu);

    // Step 3: IMU → LiDAR frame
    Eigen::Matrix<Scalar, 3, 1> p_corrected = T_lidar_imu_.inverse().transformPoint(p_corrected_imu);

    pcl::PointXYZI pt_corrected;
    pt_corrected.x = p_corrected.x();
    pt_corrected.y = p_corrected.y();
    pt_corrected.z = p_corrected.z();
    pt_corrected.intensity = point.intensity;
    deskewed->push_back(pt_corrected);
  }

  return deskewed;
}

SE3Pose PointCloudDeskewer::interpolatePose(
    const std::vector<std::pair<Timestamp, SE3Pose>>& trajectory,
    const Timestamp& query_time) const {

  if (trajectory.size() == 1) {
    return trajectory[0].second;
  }

  // Find bracketing timestamps
  std::size_t i = 0;
  for (; i < trajectory.size() - 1; ++i) {
    if (trajectory[i + 1].first > query_time) break;
  }

  // Edge cases
  if (i >= trajectory.size() - 1) {
    return trajectory.back().second;
  }

  const auto& [t0, T0] = trajectory[i];
  const auto& [t1, T1] = trajectory[i + 1];

  double total = toSeconds(t0, t1);
  if (total < 1e-9) return T0;

  double alpha = toSeconds(t0, query_time) / total;
  alpha = std::max(0.0, std::min(1.0, alpha));

  // Spherical linear interpolation for rotation
  Eigen::Quaternion<Scalar> q0 = T0.q;
  Eigen::Quaternion<Scalar> q1 = T1.q;
  Eigen::Quaternion<Scalar> q_interp = q0.slerp(static_cast<Scalar>(alpha), q1);

  // Linear interpolation for translation
  Eigen::Matrix<Scalar, 3, 1> t_interp = T0.t + static_cast<Scalar>(alpha) * (T1.t - T0.t);

  return SE3Pose{q_interp, t_interp};
}

double PointCloudDeskewer::toSeconds(const Timestamp& t0, const Timestamp& t1) const {
  return static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()) * 1e-9;
}

}  // namespace m20::lio