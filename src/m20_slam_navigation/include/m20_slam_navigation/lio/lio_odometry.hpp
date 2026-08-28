#pragma once
/**
 * @file lio_odometry.hpp
 * @brief LiDAR-Inertial Odometry (LIO) front-end nodelet.
 *
 * Pipeline:
 *   1. Buffer incoming IMU (200Hz+) and LiDAR (10-20Hz) data.
 *   2. On LiDAR scan arrival:
 *      a. Integrate IMU to get prior pose and trajectory.
 *      b. Deskew point cloud using trajectory.
 *      c. Apply the iterated point-to-plane ESKF observation update.
 *      d. Update voxel map with registered cloud.
 *      e. Publish odometry, TF, and keyframe poses.
 *      f. Send relative odometry factor to back-end.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/common/thread_safe_queue.hpp"
#include "m20_slam_navigation/lio/imu_processor.hpp"
#include "m20_slam_navigation/lio/point_cloud_deskewer.hpp"
#include "m20_slam_navigation/lio/voxel_map.hpp"
#include "m20_slam_navigation/lio/vendor_lio_eskf.hpp"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <thread>

namespace m20::lio {

/// Callback type: invoked when a new LIO odometry estimate is available
using LioOdometryCallback = std::function<void(
  const PoseWithCovariance& pose, FrameId frame_id, const Timestamp& stamp)>;

/// Callback: invoked when a keyframe is inserted (for loop closure detection)
using KeyframeCallback = std::function<void(FrameId frame_id, const SE3Pose& pose,
                                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)>;
using RegistrationFailureCallback = std::function<void(const VendorLioUpdateResult& result)>;
using AlignedCloudCallback = std::function<void(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_body,
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_world,
  FrameId frame_id, const Timestamp& stamp)>;

class LIOOdometry {
public:
  LIOOdometry(const SensorParams& sensor_params, const LIOParams& lio_params,
              const BackendParams& backend_params);

  /// Main processing loop (runs in its own thread)
  void run();

  /// Stop processing
  void stop();

  /// Feed raw point cloud (called from ROS subscriber thread)
  void addPointCloud(const LiDARPacket& pkt);

  /// Feed IMU measurement
  void addImu(const ImuPacket& imu);

  /// Set callbacks
  void setOdometryCallback(LioOdometryCallback cb) { odom_cb_ = std::move(cb); }
  void setKeyframeCallback(KeyframeCallback cb) { keyframe_cb_ = std::move(cb); }
  void setRegistrationFailureCallback(RegistrationFailureCallback cb) {
    registration_failure_cb_ = std::move(cb);
  }
  void setAlignedCloudCallback(AlignedCloudCallback cb) {
    aligned_cloud_cb_ = std::move(cb);
  }

  /// Get current pose estimate (thread-safe)
  PoseWithCovariance getCurrentPose() const;

  /// Wait for all queued LiDAR scans to finish processing.
  bool waitUntilIdle(std::chrono::milliseconds timeout) const;

  std::size_t lidarQueueSize() const { return lidar_queue_.size(); }
  std::uint64_t droppedScans() const { return dropped_scans_.load(); }
  std::uint64_t processedScans() const { return processed_scans_.load(); }
  std::uint64_t initializationWaitScans() const { return initialization_wait_scans_.load(); }
  std::uint64_t bootstrapScans() const { return bootstrap_scans_.load(); }
  std::uint64_t successfulUpdates() const { return successful_updates_.load(); }
  std::uint64_t rejectedUpdates() const { return rejected_updates_.load(); }
  std::uint64_t emptyRegistrationClouds() const { return empty_registration_clouds_.load(); }

  /// Get shared voxel map
  std::shared_ptr<VoxelMap> getVoxelMap() const { return voxel_map_; }

  /// IMU bias estimates
  Eigen::Matrix<Scalar, 3, 1> getGyroBias() const { return bg_; }
  Eigen::Matrix<Scalar, 3, 1> getAccelBias() const { return ba_; }

private:
  /// Process a single LiDAR scan
  void processScan(const LiDARPacket& pkt);

  /// Check if a new keyframe should be inserted
  bool isKeyframe(const SE3Pose& current, const SE3Pose& last_keyframe) const;

  // Parameters
  SensorParams    sensor_params_;
  LIOParams       lio_params_;
  BackendParams   backend_params_;

  // Components
  std::unique_ptr<ImuProcessor>       imu_processor_;
  std::unique_ptr<PointCloudDeskewer> deskewer_;
  std::shared_ptr<VoxelMap>          voxel_map_;
  std::unique_ptr<VendorLioEskf>     eskf_;

  // Buffers
  SPSCQueue<ImuPacket, 4096>         imu_queue_;
  MPSCQueue<LiDARPacket>             lidar_queue_;

  // State
  SE3Pose                            current_pose_;
  Eigen::Matrix<Scalar, 3, 1>        current_vel_{0, 0, 0};
  Eigen::Matrix<Scalar, 3, 1>        bg_{0, 0, 0};     ///< estimated gyro bias
  Eigen::Matrix<Scalar, 3, 1>        ba_{0, 0, 0};     ///< estimated accel bias
  std::optional<Timestamp>           state_stamp_;      ///< timestamp represented by current state
  SE3Pose                            last_keyframe_pose_;
  FrameId                            last_keyframe_id_{0};
  FrameId                            frame_counter_{0};

  // Thread safety
  mutable std::shared_mutex          pose_mutex_;
  std::atomic<bool>                  running_{false};
  std::atomic<bool>                  processing_scan_{false};
  std::atomic<std::uint64_t>         dropped_scans_{0};
  std::atomic<std::uint64_t>         processed_scans_{0};
  std::atomic<std::uint64_t>         initialization_wait_scans_{0};
  std::atomic<std::uint64_t>         bootstrap_scans_{0};
  std::atomic<std::uint64_t>         successful_updates_{0};
  std::atomic<std::uint64_t>         rejected_updates_{0};
  std::atomic<std::uint64_t>         empty_registration_clouds_{0};
  std::unique_ptr<std::thread>       worker_thread_;

  // Callbacks
  LioOdometryCallback                odom_cb_;
  KeyframeCallback                   keyframe_cb_;
  RegistrationFailureCallback        registration_failure_cb_;
  AlignedCloudCallback               aligned_cloud_cb_;
};

}  // namespace m20::lio
