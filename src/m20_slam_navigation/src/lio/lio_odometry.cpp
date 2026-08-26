#include "m20_slam_navigation/lio/lio_odometry.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <chrono>
#include <thread>

namespace m20::lio {

LIOOdometry::LIOOdometry(const SensorParams& sensor_params,
                         const LIOParams& lio_params,
                         const BackendParams& backend_params)
    : sensor_params_(sensor_params)
    , lio_params_(lio_params)
    , backend_params_(backend_params)
    , imu_processor_(std::make_unique<ImuProcessor>(sensor_params, lio_params))
    , deskewer_(std::make_unique<PointCloudDeskewer>(sensor_params.T_lidar_imu))
    , voxel_map_(std::make_shared<VoxelMap>(lio_params.voxel_size, lio_params.max_voxels))
    , vgicp_(std::make_unique<FastVAGICP>(lio_params))
{
}

void LIOOdometry::run() {
  running_ = true;

  while (running_) {
    // Pop next LiDAR frame (blocking with timeout)
    auto pkt_opt = lidar_queue_.pop_for(std::chrono::milliseconds(50));
    if (!pkt_opt) continue;

    LiDARPacket pkt = std::move(*pkt_opt);
    processScan(pkt);
  }
}

void LIOOdometry::stop() {
  running_ = false;
  if (worker_thread_ && worker_thread_->joinable()) {
    worker_thread_->join();
  }
}

void LIOOdometry::addPointCloud(const LiDARPacket& pkt) {
  lidar_queue_.push(pkt);
}

void LIOOdometry::addImu(const ImuPacket& imu) {
  imu_queue_.try_push(imu);
  // Also feed to the IMU processor
  imu_processor_->addMeasurement(imu);
}

PoseWithCovariance LIOOdometry::getCurrentPose() const {
  std::shared_lock<std::shared_mutex> lock(pose_mutex_);
  PoseWithCovariance pwc;
  pwc.pose = current_pose_;
  // Default identity covariance
  pwc.covariance = Eigen::Matrix<Scalar, 6, 6>::Identity() * 0.01;
  return pwc;
}

void LIOOdometry::processScan(const LiDARPacket& pkt) {
  using namespace std::chrono;

  frame_counter_++;
  FrameId frame_id = frame_counter_;

  // Step 1: Drain IMU queue and integrate for pose prior
  ImuProcessor::IntegrationResult imu_result;
  {
    // Use last known pose and velocity for integration start
    SE3Pose pose_prior = current_pose_;
    Eigen::Matrix<Scalar, 3, 1> vel_prior = current_vel_;

    // Find IMU buffer range for this scan
    auto imu_data = imu_processor_->getMeasurementsBetween(
        pkt.stamp, pkt.stamp);  // TODO: use actual scan start/end

    if (!imu_data.empty()) {
      Timestamp t_start = imu_data.front().stamp;
      Timestamp t_end   = imu_data.back().stamp;
      imu_result = imu_processor_->integrate(t_start, t_end, pose_prior, vel_prior, ba_, bg_);
    } else {
      // No IMU: use last pose directly
      imu_result.pose_end = pose_prior;
      imu_result.velocity_end = vel_prior;
      imu_result.trajectory.push_back({pkt.stamp, pose_prior});
      imu_result.gravity_direction = pose_prior.q.conjugate()._transformVector(lio_params_.gravity);
    }
  }

  // Step 2: Deskew point cloud
  auto deskewed = deskewer_->deskew(pkt.cloud, imu_result.trajectory, pkt.stamp);

  // Step 3: Downsample for registration speed
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setLeafSize(0.1, 0.1, 0.1);  // 10cm for speed
  vg.setInputCloud(deskewed);
  auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  vg.filter(*downsampled);

  // Step 4: FastVAGICP registration to voxel map
  VGICPResult reg_result = vgicp_->align(downsampled, voxel_map_, imu_result.pose_end);

  if (reg_result.converged) {
    // Step 5: Update state
    {
      std::unique_lock<std::shared_mutex> lock(pose_mutex_);
      current_pose_ = reg_result.T_world_lidar;
      current_vel_  = imu_result.velocity_end;

      // Gravity alignment: slowly update IMU biases from gravity direction
      // Simplified: just track the gravity direction for the back-end
    }

    // Step 6: Update voxel map with new scan
    voxel_map_->insertCloud(downsampled, reg_result.T_world_lidar);

    // Step 7: Publish odometry
    if (odom_cb_) {
      PoseWithCovariance pwc;
      pwc.pose = reg_result.T_world_lidar;
      pwc.covariance = reg_result.information.inverse();
      // Clamp diagonal for safety
      for (int i = 0; i < 6; ++i) {
        pwc.covariance(i, i) = math::clamp(pwc.covariance(i, i), Scalar(1e-6), Scalar(100.0));
      }
      odom_cb_(pwc, frame_id);
    }

    // Step 8: Check if keyframe
    if (isKeyframe(reg_result.T_world_lidar, last_keyframe_pose_) || frame_id == 1) {
      last_keyframe_pose_ = reg_result.T_world_lidar;
      last_keyframe_id_ = frame_id;

      if (keyframe_cb_) {
        keyframe_cb_(frame_id, reg_result.T_world_lidar, deskewed);
      }
    }
  } else {
    // Registration failed: fall back to IMU-only prediction
    std::unique_lock<std::shared_mutex> lock(pose_mutex_);
    current_pose_ = imu_result.pose_end;
    current_vel_  = imu_result.velocity_end;
  }
}

bool LIOOdometry::isKeyframe(const SE3Pose& current, const SE3Pose& last_keyframe) const {
  // Translation distance
  Scalar trans_dist = (current.t - last_keyframe.t).norm();
  if (trans_dist > lio_params_.keyframe_distance) return true;

  // Rotation angle
  Eigen::Quaternion<Scalar> dq = last_keyframe.q.conjugate() * current.q;
  Scalar angle = 2.0 * std::acos(math::clamp(std::abs(dq.w()), Scalar(0), Scalar(1)));
  if (angle > lio_params_.keyframe_angle) return true;

  return false;
}

}  // namespace m20::lio