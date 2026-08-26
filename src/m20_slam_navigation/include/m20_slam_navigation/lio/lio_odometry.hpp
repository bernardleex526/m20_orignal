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
 *      c. Register deskewed cloud to incremental voxel map via FastVAGICP.
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
#include "m20_slam_navigation/lio/fast_vgicp.hpp"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>

namespace m20::lio {

/// Callback type: invoked when a new LIO odometry estimate is available
using LioOdometryCallback = std::function<void(const PoseWithCovariance& pose, FrameId frame_id)>;

/// Callback: invoked when a keyframe is inserted (for loop closure detection)
using KeyframeCallback = std::function<void(FrameId frame_id, const SE3Pose& pose,
                                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)>;

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

  /// Get current pose estimate (thread-safe)
  PoseWithCovariance getCurrentPose() const;

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
  std::unique_ptr<FastVAGICP>        vgicp_;

  // Buffers
  SPSCQueue<ImuPacket, 4096>         imu_queue_;
  MPSCQueue<LiDARPacket>             lidar_queue_;

  // State
  SE3Pose                            current_pose_;
  Eigen::Matrix<Scalar, 3, 1>        current_vel_{0, 0, 0};
  Eigen::Matrix<Scalar, 3, 1>        bg_{0, 0, 0};     ///< estimated gyro bias
  Eigen::Matrix<Scalar, 3, 1>        ba_{0, 0, 0};     ///< estimated accel bias
  SE3Pose                            last_keyframe_pose_;
  FrameId                            last_keyframe_id_{0};
  FrameId                            frame_counter_{0};

  // Thread safety
  mutable std::shared_mutex          pose_mutex_;
  std::atomic<bool>                  running_{false};
  std::unique_ptr<std::thread>       worker_thread_;

  // Callbacks
  LioOdometryCallback                odom_cb_;
  KeyframeCallback                   keyframe_cb_;
};

}  // namespace m20::lio