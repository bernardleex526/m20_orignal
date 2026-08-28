#include "m20_slam_navigation/lio/vendor_lio_eskf.hpp"

#include "m20_slam_navigation/common/math_utils.hpp"

#include <chrono>
#include <cmath>
#include <limits>

namespace m20::lio {

namespace {
constexpr int kPos = 0;
constexpr int kRot = 3;
constexpr int kExtRot = 6;
constexpr int kExtPos = 9;
constexpr int kVel = 12;
constexpr int kGyroBias = 15;
constexpr int kAccelBias = 18;
constexpr int kGravity = 21;

Eigen::Matrix<Scalar, 3, 2> makeGravityTangentBasis(
    const Eigen::Matrix<Scalar, 3, 1>& gravity) {
  Eigen::Matrix<Scalar, 3, 1> direction = gravity;
  if (direction.norm() < 1e-9) direction = Eigen::Vector3d(0.0, 0.0, -1.0);
  direction.normalize();
  const Eigen::Matrix<Scalar, 3, 1> seed =
    std::abs(direction.z()) < Scalar(0.9) ? Eigen::Vector3d::UnitZ() :
                                           Eigen::Vector3d::UnitX();
  Eigen::Matrix<Scalar, 3, 2> basis;
  basis.col(0) = (seed - direction * direction.dot(seed)).normalized();
  basis.col(1) = direction.cross(basis.col(0)).normalized();
  return basis;
}
}

VendorLioEskf::VendorLioEskf(
    const SensorParams& sensor_params, const LIOParams& lio_params)
    : sensor_params_(sensor_params), lio_params_(lio_params) {
  state_.gravity = lio_params_.gravity;
}

void VendorLioEskf::initialize(
    const SE3Pose& pose,
    const Eigen::Matrix<Scalar, 3, 1>& velocity,
    const Eigen::Matrix<Scalar, 3, 1>& gyro_bias,
    const Eigen::Matrix<Scalar, 3, 1>& accel_bias) {
  state_.pose = pose;
  state_.lidar_in_imu = sensor_params_.T_lidar_imu;
  state_.velocity = velocity;
  state_.gyro_bias = gyro_bias;
  state_.accel_bias = accel_bias;
  state_.gravity = lio_params_.gravity;
  state_.covariance.setIdentity();
  // dr_lio keeps the extrinsics in the 23-DOF manifold even when online
  // estimation is disabled. Keep those blocks tightly constrained.
  state_.covariance.block<3, 3>(kExtRot, kExtRot) *= 1e-5;
  state_.covariance.block<3, 3>(kExtPos, kExtPos) *= 1e-5;
  state_.covariance.block<3, 3>(kGyroBias, kGyroBias) *= 1e-4;
  state_.covariance.block<3, 3>(kAccelBias, kAccelBias) *= 1e-3;
  state_.covariance.block<2, 2>(kGravity, kGravity) *= 1e-5;
  initialized_ = true;
}

void VendorLioEskf::setPredictedNominal(
    const SE3Pose& pose, const Eigen::Matrix<Scalar, 3, 1>& velocity) {
  state_.pose = pose;
  state_.velocity = velocity;
}

void VendorLioEskf::propagateCovariance(
    const std::vector<ImuPacket>& measurements) {
  if (!initialized_ || measurements.size() < 2U) return;
  using Covariance = VendorLioState::Covariance;
  for (std::size_t index = 0; index + 1U < measurements.size(); ++index) {
    const auto& first = measurements[index];
    const auto& second = measurements[index + 1U];
    const Scalar dt = std::chrono::duration_cast<std::chrono::nanoseconds>(
      second.stamp - first.stamp).count() * Scalar(1e-9);
    if (!(dt > 0.0) || dt > 0.1) continue;

    const Eigen::Matrix<Scalar, 3, 1> gyro =
      Scalar(0.5) * (first.gyro + second.gyro) - state_.gyro_bias;
    const Eigen::Matrix<Scalar, 3, 1> accel =
      Scalar(0.5) * (first.accel + second.accel) - state_.accel_bias;
    const Eigen::Matrix<Scalar, 3, 3> rotation = state_.pose.q.toRotationMatrix();

    Covariance F = Covariance::Zero();
    F.block<3, 3>(kPos, kVel).setIdentity();
    F.block<3, 3>(kRot, kRot) = -math::skew(gyro);
    F.block<3, 3>(kRot, kGyroBias) = -Eigen::Matrix<Scalar, 3, 3>::Identity();
    F.block<3, 3>(kVel, kRot) = -rotation * math::skew(accel);
    F.block<3, 3>(kVel, kAccelBias) = -rotation;
    F.block<3, 2>(kVel, kGravity) = gravityTangentBasis();

    Eigen::Matrix<Scalar, VendorLioState::kErrorDim, 12> G =
      Eigen::Matrix<Scalar, VendorLioState::kErrorDim, 12>::Zero();
    G.block<3, 3>(kRot, 0) = -Eigen::Matrix<Scalar, 3, 3>::Identity();
    G.block<3, 3>(kVel, 3) = -rotation;
    G.block<3, 3>(kGyroBias, 6).setIdentity();
    G.block<3, 3>(kAccelBias, 9).setIdentity();
    Eigen::Matrix<Scalar, 12, 12> Q = Eigen::Matrix<Scalar, 12, 12>::Zero();
    Q.block<3, 3>(0, 0).diagonal().setConstant(lio_params_.gyr_cov);
    Q.block<3, 3>(3, 3).diagonal().setConstant(lio_params_.acc_cov);
    Q.block<3, 3>(6, 6).diagonal().setConstant(lio_params_.b_gyr_cov);
    Q.block<3, 3>(9, 9).diagonal().setConstant(lio_params_.b_acc_cov);

    const Covariance transition = Covariance::Identity() + F * dt;
    state_.covariance = transition * state_.covariance * transition.transpose() +
      G * Q * G.transpose() * dt;
    state_.covariance = Scalar(0.5) *
      (state_.covariance + state_.covariance.transpose());
  }
}

VendorLioEskf::ErrorVector VendorLioEskf::boxMinus(
    const VendorLioState& reference) const {
  ErrorVector error = ErrorVector::Zero();
  error.segment<3>(kPos) = state_.pose.t - reference.pose.t;
  error.segment<3>(kRot) = math::so3_log(
    (reference.pose.q.conjugate() * state_.pose.q).toRotationMatrix());
  error.segment<3>(kExtRot) = math::so3_log(
    (reference.lidar_in_imu.q.conjugate() * state_.lidar_in_imu.q).toRotationMatrix());
  error.segment<3>(kExtPos) =
    state_.lidar_in_imu.t - reference.lidar_in_imu.t;
  error.segment<3>(kVel) = state_.velocity - reference.velocity;
  error.segment<3>(kGyroBias) = state_.gyro_bias - reference.gyro_bias;
  error.segment<3>(kAccelBias) = state_.accel_bias - reference.accel_bias;
  error.segment<2>(kGravity) =
    makeGravityTangentBasis(reference.gravity).transpose() *
    (state_.gravity - reference.gravity);
  return error;
}

Eigen::Matrix<Scalar, 3, 2> VendorLioEskf::gravityTangentBasis() const {
  return makeGravityTangentBasis(state_.gravity);
}

void VendorLioEskf::applyError(const ErrorVector& error) {
  state_.pose.t += error.segment<3>(kPos);
  state_.pose.q = state_.pose.q * Eigen::Quaternion<Scalar>(
    math::so3_exp(error.segment<3>(kRot)));
  state_.pose.q.normalize();
  if (lio_params_.extrinsic_est_en) {
    state_.lidar_in_imu.q = state_.lidar_in_imu.q * Eigen::Quaternion<Scalar>(
      math::so3_exp(error.segment<3>(kExtRot)));
    state_.lidar_in_imu.q.normalize();
    state_.lidar_in_imu.t += error.segment<3>(kExtPos);
  }
  state_.velocity += error.segment<3>(kVel);
  state_.gyro_bias += error.segment<3>(kGyroBias);
  state_.accel_bias += error.segment<3>(kAccelBias);
  state_.gravity += gravityTangentBasis() * error.segment<2>(kGravity);
  const Scalar gravity_norm = lio_params_.gravity.norm();
  if (gravity_norm > 0.0 && state_.gravity.norm() > 1e-6) {
    state_.gravity = state_.gravity.normalized() * gravity_norm;
  }
}

VendorLioUpdateResult VendorLioEskf::update(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_lidar,
    const std::shared_ptr<VoxelMap>& voxel_map) {
  const auto begin = std::chrono::steady_clock::now();
  VendorLioUpdateResult result;
  result.T_world_imu = state_.pose;
  if (!initialized_ || !cloud_lidar || cloud_lidar->empty() || !voxel_map ||
      voxel_map->size() == 0U) {
    return result;
  }

  const VendorLioState predicted = state_;
  const StateMatrix prior_covariance = state_.covariance +
    StateMatrix::Identity() * Scalar(1e-9);
  const StateMatrix prior_information = prior_covariance.ldlt().solve(
    StateMatrix::Identity());
  StateMatrix final_information = prior_information;
  Scalar previous_update = std::numeric_limits<Scalar>::infinity();
  std::vector<Eigen::Matrix<Scalar, 3, 1>> points_imu;
  points_imu.reserve(cloud_lidar->size());
  const Eigen::Matrix<Scalar, 3, 3> R_il =
    state_.lidar_in_imu.q.toRotationMatrix();
  for (const auto& point : cloud_lidar->points) {
    points_imu.emplace_back(
      R_il * Eigen::Matrix<Scalar, 3, 1>(point.x, point.y, point.z) +
      state_.lidar_in_imu.t);
  }

  for (int iteration = 0; iteration < std::max(1, lio_params_.max_iterations); ++iteration) {
    StateMatrix information = prior_information;
    ErrorVector gradient = prior_information * boxMinus(predicted);
    Eigen::Matrix<Scalar, 6, 6> pose_information =
      Eigen::Matrix<Scalar, 6, 6>::Zero();
    int correspondences = 0;
    Scalar residual_sum = 0.0;
    const Eigen::Matrix<Scalar, 3, 3> R_wi = state_.pose.q.toRotationMatrix();
    std::vector<Eigen::Matrix<Scalar, 3, 1>> points_world;
    points_world.reserve(points_imu.size());
    for (const auto& point : points_imu) {
      points_world.emplace_back(R_wi * point + state_.pose.t);
    }
    std::vector<VoxelMap::PlaneMatch> matches;
    std::vector<std::uint8_t> valid;
    voxel_map->findPlanes(points_world, matches, valid);

    for (std::size_t point_index = 0; point_index < points_imu.size(); ++point_index) {
      if (valid[point_index] == 0U) continue;
      const auto& p_imu = points_imu[point_index];
      const auto& match = matches[point_index];
      if (!match.normal.allFinite() || match.normal.squaredNorm() < Scalar(1e-12) ||
        !std::isfinite(match.residual)) {
        continue;
      }

      Eigen::Matrix<Scalar, 1, VendorLioState::kErrorDim> H =
        Eigen::Matrix<Scalar, 1, VendorLioState::kErrorDim>::Zero();
      H.block<1, 3>(0, kPos) = match.normal.transpose();
      H.block<1, 3>(0, kRot) =
        -match.normal.transpose() * R_wi * math::skew(p_imu);
      // Native ObsModel exposes a dynamic h_x with exactly 12 columns:
      // position, attitude, LiDAR-in-IMU rotation, LiDAR-in-IMU translation.
      if (lio_params_.extrinsic_est_en) {
        const Eigen::Matrix<Scalar, 3, 1> p_lidar(
          cloud_lidar->points[point_index].x,
          cloud_lidar->points[point_index].y,
          cloud_lidar->points[point_index].z);
        H.block<1, 3>(0, kExtRot) =
          -match.normal.transpose() * R_wi * R_il * math::skew(p_lidar);
        H.block<1, 3>(0, kExtPos) = match.normal.transpose() * R_wi;
      }
      // The native contract exposes a scalar lidar_cov for each accepted
      // point-to-plane observation. VoxelMap already applies the native
      // esti_plane_threshold gate before this update.
      const Scalar inverse_noise = Scalar(1.0) /
        std::max(Scalar(1e-9), lio_params_.lidar_cov);
      information.noalias() += inverse_noise * H.transpose() * H;
      gradient.noalias() += inverse_noise * H.transpose() * match.residual;

      Eigen::Matrix<Scalar, 1, 6> H_pose;
      H_pose << H.block<1, 3>(0, kRot), H.block<1, 3>(0, kPos);
      pose_information.noalias() += inverse_noise * H_pose.transpose() * H_pose;
      residual_sum += std::abs(match.residual);
      ++correspondences;
    }

    result.iterations = iteration + 1;
    result.correspondences = correspondences;
    result.mean_residual = correspondences > 0 ?
      residual_sum / static_cast<Scalar>(correspondences) : Scalar(0.0);
    // The native ObsModel sets valid=false only when effect_feat_num <= 0.
    if (correspondences == 0) break;

    Eigen::LDLT<StateMatrix> decomposition(information);
    if (decomposition.info() != Eigen::Success) break;
    const ErrorVector update = -decomposition.solve(gradient);
    if (!update.allFinite()) break;
    applyError(update);
    result.final_update_norm = update.head<6>().norm();
    final_information = information;
    result.information = pose_information +
      Eigen::Matrix<Scalar, 6, 6>::Identity() * Scalar(1e-9);
    constexpr Scalar kIkfomCompatibleUpdateTolerance = 1e-3;
    if (result.final_update_norm < kIkfomCompatibleUpdateTolerance ||
        std::abs(previous_update - result.final_update_norm) <
          kIkfomCompatibleUpdateTolerance * Scalar(0.1)) {
      result.converged = true;
      break;
    }
    previous_update = result.final_update_norm;
    result.converged = iteration + 1 == std::max(1, lio_params_.max_iterations);
  }

  if (result.correspondences > 0 && final_information.allFinite()) {
    state_.covariance = final_information.ldlt().solve(StateMatrix::Identity());
    state_.covariance = Scalar(0.5) *
      (state_.covariance + state_.covariance.transpose());
  } else {
    state_ = predicted;
    result.converged = false;
  }
  result.T_world_imu = state_.pose;
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
  return result;
}

}  // namespace m20::lio
