#include "m20_slam_navigation/localization/relocalizer.hpp"

#include <algorithm>

namespace m20::localization {

Relocalizer::Relocalizer(const LocalizationParams& localization_params,
                         const SensorParams& sensor_params)
    : loc_params_(localization_params)
    , sensor_params_(sensor_params)
    , map_loader_(std::make_unique<PriorMapLoader>(localization_params))
    , ndt_matcher_(std::make_unique<NDTMatcher>(localization_params))
    , eskf_(std::make_unique<ESKF>(localization_params)) {
}

bool Relocalizer::loadMap(const std::string& map_path) {
  bool ok = map_loader_->loadMap(map_path);
  if (ok) {
    ndt_matcher_->setTargetMap(map_loader_->getDownsampledMap());
  }
  return ok;
}

bool Relocalizer::relocalize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const PoseWithCovariance& initial_guess) {

  std::lock_guard<std::mutex> lock(mutex_);

  if (!map_loader_->isLoaded()) return false;

  // Try single-hypothesis NDT first with initial guess
  NDTResult ndt_result = ndt_matcher_->align(cloud, initial_guess.pose);

  if (ndt_result.converged && ndt_result.fitness_score < 5.0) {
    // Good match: initialize ESKF
    eskf_->initialize(ndt_result.T_world_lidar);
    localized_ = true;
    last_ndt_pose_ = ndt_result.T_world_lidar;

    if (reloc_cb_) {
      reloc_cb_(ndt_result.T_world_lidar, true);
    }
    return true;
  }

  // Multi-hypothesis global search
  auto bounds = map_loader_->getBounds();
  Eigen::Matrix<Scalar, 3, 1> search_center = initial_guess.pose.t;
  Scalar search_radius = loc_params_.hypothesis_trans_range;

  ndt_result = ndt_matcher_->globalRelocalize(
      cloud, search_center, search_radius,
      loc_params_.num_hypotheses);

  if (ndt_result.converged && ndt_result.fitness_score < 5.0) {
    eskf_->initialize(ndt_result.T_world_lidar);
    localized_ = true;
    last_ndt_pose_ = ndt_result.T_world_lidar;

    if (reloc_cb_) {
      reloc_cb_(ndt_result.T_world_lidar, true);
    }
    return true;
  }

  localized_ = false;
  if (reloc_cb_) {
    reloc_cb_(initial_guess.pose, false);
  }
  return false;
}

void Relocalizer::predict(const ImuPacket& imu) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!localized_) return;

  // Compute dt from last prediction (simplified: use nominal 200Hz)
  Scalar dt = 1.0 / sensor_params_.imu_hz;
  eskf_->predict(imu.accel, imu.gyro, dt);
}

void Relocalizer::updateOdometry(const FootOdomPacket& odom) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!localized_) return;

  Eigen::Matrix<Scalar, 6, 6> R_odom = odom.covariance;
  // Inflate covariance for slip uncertainty
  for (int i = 0; i < 6; ++i) {
    R_odom(i, i) *= (1.0 + sensor_params_.odom_slip_ratio);
  }

  eskf_->updateOdometry(odom.pose, R_odom);
  eskf_->injectErrorAndReset();
}

void Relocalizer::updateNDT(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!localized_ || !map_loader_->isLoaded()) return;

  // Use current ESKF pose as initial guess
  SE3Pose T_init = eskf_->getPose();

  NDTResult ndt_result = ndt_matcher_->align(cloud, T_init);

  if (ndt_result.converged && ndt_result.fitness_score < 5.0) {
    last_ndt_pose_ = ndt_result.T_world_lidar;

    // Build measurement covariance from NDT fitness
    Eigen::Matrix<Scalar, 6, 6> R_ndt = Eigen::Matrix<Scalar, 6, 6>::Identity();
    R_ndt.block<3, 3>(0, 0) *= loc_params_.eskf_ndt_rot_noise * ndt_result.fitness_score;
    R_ndt.block<3, 3>(3, 3) *= loc_params_.eskf_ndt_pos_noise * ndt_result.fitness_score;

    eskf_->updateNDT(ndt_result.T_world_lidar, R_ndt);
    eskf_->injectErrorAndReset();
  }
}

PoseWithCovariance Relocalizer::getCurrentPose() const {
  std::lock_guard<std::mutex> lock(mutex_);
  PoseWithCovariance pwc;
  if (localized_) {
    pwc.pose = eskf_->getPose();
    pwc.covariance = eskf_->getCovariance().block<6, 6>(0, 0);  // pose covariance block
  }
  return pwc;
}

}  // namespace m20::localization