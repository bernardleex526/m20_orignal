#include "m20_slam_navigation/lio/lio_odometry.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"
#include "m20_slam_navigation/lio/vendor_output_contract.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
    , voxel_map_(std::make_shared<VoxelMap>(lio_params))
    , eskf_(std::make_unique<VendorLioEskf>(sensor_params, lio_params))
{
}

void LIOOdometry::run() {
  running_ = true;

  while (running_) {
    // Pop next LiDAR frame (blocking with timeout)
    auto pkt_opt = lidar_queue_.pop_for(std::chrono::milliseconds(50));
    if (!pkt_opt) continue;

    LiDARPacket pkt = std::move(*pkt_opt);
    processing_scan_ = true;
    processScan(pkt);
    ++processed_scans_;
    processing_scan_ = false;
  }
}

void LIOOdometry::stop() {
  running_ = false;
  if (worker_thread_ && worker_thread_->joinable()) {
    worker_thread_->join();
  }
}

void LIOOdometry::addPointCloud(const LiDARPacket& pkt) {
  const auto maximum = static_cast<std::size_t>(std::max(1, lio_params_.max_lidar_queue_size));
  while (lidar_queue_.size() >= maximum) {
    if (!lidar_queue_.try_pop()) break;
    ++dropped_scans_;
  }
  lidar_queue_.push(pkt);
}

void LIOOdometry::addImu(const ImuPacket& imu) {
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

bool LIOOdometry::waitUntilIdle(std::chrono::milliseconds timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (lidar_queue_.empty() && !processing_scan_.load()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return lidar_queue_.empty() && !processing_scan_.load();
}

void LIOOdometry::processScan(const LiDARPacket& pkt) {
  using namespace std::chrono;

  if (!pkt.cloud || pkt.cloud->empty()) {
    return;
  }

  frame_counter_++;
  FrameId frame_id = frame_counter_;

  // Step 1: propagate continuously to the scan end.  The public vendor
  // ImuProcess contract deskews into the end frame and leaves the ESKF state
  // at lidar_end_time_, so registration must use the same timestamp.
  ImuProcessor::IntegrationResult deskew_result;
  SE3Pose registration_prior;
  Eigen::Matrix<Scalar, 3, 1> velocity_at_scan = current_vel_;
  std::vector<ImuPacket> covariance_imus;
  {
    SE3Pose pose_at_scan_start = current_pose_;
    Eigen::Matrix<Scalar, 3, 1> velocity_at_scan_start = current_vel_;

    // The corrected state is associated with the previous scan end.  Bridge
    // the inter-scan interval first, then retain the in-scan trajectory for
    // end-frame point compensation.
    if (state_stamp_ && *state_stamp_ < pkt.stamp) {
      const auto inter_scan = imu_processor_->integrate(
        *state_stamp_, pkt.stamp, current_pose_, current_vel_, ba_, bg_);
      pose_at_scan_start = inter_scan.pose_end;
      velocity_at_scan_start = inter_scan.velocity_end;
    }
    deskew_result = imu_processor_->integrate(
      pkt.stamp, pkt.scan_end, pose_at_scan_start, velocity_at_scan_start, ba_, bg_);
    registration_prior = deskew_result.pose_end;
    velocity_at_scan = deskew_result.velocity_end;
    if (state_stamp_) {
      covariance_imus = imu_processor_->getMeasurementsBetween(*state_stamp_, pkt.scan_end);
    } else {
      covariance_imus = imu_processor_->getMeasurementsBetween(pkt.stamp, pkt.scan_end);
    }
  }

  // Apply the vendor skip_num contract before deskewing so the expensive IMU
  // interpolation and registration stages both see the reduced point set.
  pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<double> sampled_offsets;
  std::vector<std::uint16_t> sampled_rings;
  const auto stride = static_cast<std::size_t>(std::max(1, lio_params_.point_stride));
  const auto sampled_size = (pkt.cloud->size() + stride - 1U) / stride;
  sampled_cloud->reserve(sampled_size);
  sampled_offsets.reserve(sampled_size);
  sampled_rings.reserve(sampled_size);
  for (std::size_t index = 0; index < pkt.cloud->size(); index += stride) {
    sampled_cloud->push_back(pkt.cloud->points[index]);
    sampled_offsets.push_back(
      index < pkt.point_time_offsets.size() ? pkt.point_time_offsets[index] : 0.0);
    sampled_rings.push_back(index < pkt.rings.size() ? pkt.rings[index] : 0U);
  }

  // Step 2: Deskew point cloud
  auto deskewed = deskewer_->deskew(
      sampled_cloud, sampled_offsets, deskew_result.trajectory, pkt.stamp);

  const auto products = makeVendorCloudProducts(
    deskewed, sampled_offsets, sampled_rings,
    sensor_params_.lidar_min_range, sensor_params_.lidar_max_range,
    lio_params_.enable_downsample, lio_params_.downsample_leaf_size,
    lio_params_.leaf_size_body);
  const auto & body_output = products.body_cloud;
  const auto & registration_output = products.registration_cloud;
  const auto & downsampled = products.registration_points;

  if (downsampled->empty()) {
    ++empty_registration_clouds_;
    return;
  }

  // Bootstrap: scan-to-map registration cannot produce correspondences until
  // the first scan has initialized the incremental voxel map.
  if (voxel_map_->size() == 0) {
    SE3Pose initial_pose = SE3Pose::Identity();
    const auto init_window = duration_cast<nanoseconds>(
      duration<double>(std::max(0.0, lio_params_.init_time)));
    const auto init_imus = imu_processor_->getMeasurementsUpTo(pkt.scan_end);
    const auto required_samples = static_cast<std::size_t>(
      std::max(1, lio_params_.imu_init_samples));
    if (init_imus.size() < required_samples ||
      (lio_params_.init_time > 0.0 &&
       (init_imus.size() < 2 ||
        init_imus.back().stamp - init_imus.front().stamp < init_window)))
    {
      ++initialization_wait_scans_;
      return;
    }
    if (init_imus.size() >= 2) {
      Eigen::Matrix<Scalar, 3, 1> mean_accel = Eigen::Matrix<Scalar, 3, 1>::Zero();
      Eigen::Matrix<Scalar, 3, 1> mean_gyro = Eigen::Matrix<Scalar, 3, 1>::Zero();
      for (const auto & imu : init_imus) {
        mean_accel += imu.accel;
        mean_gyro += imu.gyro;
      }
      mean_accel /= static_cast<Scalar>(init_imus.size());
      mean_gyro /= static_cast<Scalar>(init_imus.size());
      initial_pose.q = imu_processor_->estimateInitialAttitude(init_imus, init_imus.size());
      bg_ = mean_gyro;
      const auto expected_specific_force =
        initial_pose.q.conjugate()._transformVector(-lio_params_.gravity);
      ba_ = mean_accel - expected_specific_force;
    }
    {
      std::unique_lock<std::shared_mutex> lock(pose_mutex_);
      current_pose_ = initial_pose;
      current_vel_.setZero();
      state_stamp_ = pkt.scan_end;
    }
    eskf_->initialize(initial_pose, current_vel_, bg_, ba_);
    const SE3Pose initial_lidar_pose = initial_pose * sensor_params_.T_lidar_imu;
    voxel_map_->insertCloud(downsampled, initial_lidar_pose);
    ++bootstrap_scans_;

    if (odom_cb_) {
      PoseWithCovariance pwc;
      pwc.pose = initial_lidar_pose;
      pwc.covariance = Eigen::Matrix<Scalar, 6, 6>::Identity() * 1e-3;
      odom_cb_(pwc, frame_id, pkt.scan_end);
    }
    last_keyframe_pose_ = initial_lidar_pose;
    last_keyframe_id_ = frame_id;
    if (keyframe_cb_) {
      keyframe_cb_(frame_id, initial_lidar_pose, downsampled);
    }
    if (aligned_cloud_cb_) {
      aligned_cloud_cb_(body_output,
                        makeVendorAlignedCloud(registration_output, initial_lidar_pose, nullptr),
                        frame_id, pkt.scan_end);
    }
    return;
  }

  // Step 4: vendor-style iterated point-to-plane ESKF update.
  eskf_->setPredictedNominal(registration_prior, velocity_at_scan);
  eskf_->propagateCovariance(covariance_imus);
  VendorLioUpdateResult reg_result = eskf_->update(downsampled, voxel_map_);

  if (reg_result.converged) {
    ++successful_updates_;
    const auto& corrected = eskf_->state();
    const SE3Pose T_world_lidar = corrected.pose * sensor_params_.T_lidar_imu;
    {
      std::unique_lock<std::shared_mutex> lock(pose_mutex_);
      current_pose_ = corrected.pose;
      current_vel_ = corrected.velocity;
      bg_ = corrected.gyro_bias;
      ba_ = corrected.accel_bias;
      state_stamp_ = pkt.scan_end;
    }

    const auto aligned_output =
      makeVendorAlignedCloud(registration_output, T_world_lidar, voxel_map_);
    voxel_map_->insertCloud(downsampled, T_world_lidar);

    if (odom_cb_) {
      PoseWithCovariance pwc;
      pwc.pose = T_world_lidar;
      const auto regularized_information = reg_result.information +
        Eigen::Matrix<Scalar, 6, 6>::Identity() * Scalar(1e-6);
      pwc.covariance = regularized_information.inverse();
      // Clamp diagonal for safety
      for (int i = 0; i < 6; ++i) {
        pwc.covariance(i, i) = math::clamp(pwc.covariance(i, i), Scalar(1e-6), Scalar(100.0));
      }
      odom_cb_(pwc, frame_id, pkt.scan_end);
    }

    if (isKeyframe(T_world_lidar, last_keyframe_pose_) || frame_id == 1) {
      last_keyframe_pose_ = T_world_lidar;
      last_keyframe_id_ = frame_id;

      if (keyframe_cb_) {
        keyframe_cb_(frame_id, T_world_lidar, deskewed);
      }
    }
    if (aligned_cloud_cb_) {
      aligned_cloud_cb_(body_output, aligned_output,
                        frame_id, pkt.scan_end);
    }
  } else {
    ++rejected_updates_;
    // Keep the IMU-predicted ESKF state when too few planar observations are
    // available (for example, during map bootstrap or severe occlusion).
    const auto& predicted = eskf_->state();
    std::unique_lock<std::shared_mutex> lock(pose_mutex_);
    current_pose_ = predicted.pose;
    current_vel_ = predicted.velocity;
    bg_ = predicted.gyro_bias;
    ba_ = predicted.accel_bias;
    state_stamp_ = pkt.scan_end;
    if (registration_failure_cb_) {
      registration_failure_cb_(reg_result);
    }
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
