#pragma once
/**
 * @file point_cloud_deskewer.hpp
 * @brief Motion distortion compensation for LiDAR scans on quadruped robots.
 *
 * During trotting/galloping, the robot body experiences significant pitch/roll
 * oscillation within a single LiDAR scan (10-20Hz). Each point in a scan has a
 * different timestamp; this module corrects each point back to the scan start
 * timestamp using the IMU-propagated trajectory.
 *
 * Method:
 *   For each point p_i with timestamp t_i ∈ [t_start, t_end]:
 *     1. Interpolate IMU trajectory to get T_i (pose at t_i).
 *     2. Transform p_i from LiDAR frame → IMU frame → world frame at t_i → world
 *        frame at t_start → back to LiDAR frame at t_start.
 *   So: p_i^corrected = T_lidar_imu⁻¹ · T_start⁻¹ · T_i · T_lidar_imu · p_i
 *
 *   In practice, for simplicity and speed, we use:
 *     ΔT = T_start⁻¹ ∘ T_i          (relative motion from t_i to t_start)
 *     p_i^corrected = T_lidar_imu⁻¹ · ΔT · T_lidar_imu · p_i
 */

#include "m20_slam_navigation/common/types.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <vector>

namespace m20::lio {

class PointCloudDeskewer {
public:
  /// @param T_lidar_imu  Extrinsic: LiDAR pose expressed in IMU frame
  explicit PointCloudDeskewer(const SE3Pose& T_lidar_imu = SE3Pose::Identity());

  /**
   * @brief Deskew raw point cloud using IMU trajectory.
   *
   * @param raw_cloud    Input point cloud with per-point timestamps (in pcl::PointXYZI intensity field as time offset [s]).
   * @param trajectory   IMU-propagated trajectory: list of (timestamp, SE3Pose) from ImuProcessor.
   * @param scan_start   Timestamp of scan start.
   * @return             Deskewed point cloud.
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr deskew(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_cloud,
      const std::vector<std::pair<Timestamp, SE3Pose>>& trajectory,
      const Timestamp& scan_start);

  /// Set LiDAR-IMU extrinsic
  void setExtrinsic(const SE3Pose& T_lidar_imu) { T_lidar_imu_ = T_lidar_imu; }

private:
  /// Interpolate pose at query time from discrete trajectory
  SE3Pose interpolatePose(const std::vector<std::pair<Timestamp, SE3Pose>>& trajectory,
                          const Timestamp& query_time) const;

  /// Convert timestamp duration to seconds
  double toSeconds(const Timestamp& t0, const Timestamp& t1) const;

  SE3Pose T_lidar_imu_;
};

}  // namespace m20::lio