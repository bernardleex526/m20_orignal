#pragma once

#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/lio/voxel_map.hpp"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace m20::lio {

/// Error-state layout used by dr_lio's public IKFoM contract:
/// [p(3), R(3), R_L_I(3), T_L_I(3), v(3), bg(3), ba(3), g(S2,2)].
/// The nominal gravity remains a three-vector with fixed norm, while its local
/// perturbation has the two tangent degrees of freedom used by MTK::S2.
struct VendorLioState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int kErrorDim = 23;
  using Covariance = Eigen::Matrix<Scalar, kErrorDim, kErrorDim>;

  SE3Pose pose;
  SE3Pose lidar_in_imu;
  Eigen::Matrix<Scalar, 3, 1> velocity{Eigen::Matrix<Scalar, 3, 1>::Zero()};
  Eigen::Matrix<Scalar, 3, 1> gyro_bias{Eigen::Matrix<Scalar, 3, 1>::Zero()};
  Eigen::Matrix<Scalar, 3, 1> accel_bias{Eigen::Matrix<Scalar, 3, 1>::Zero()};
  Eigen::Matrix<Scalar, 3, 1> gravity{0.0, 0.0, -9.81007};
  Covariance covariance{Covariance::Identity()};
};

struct VendorLioUpdateResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool converged{false};
  int correspondences{0};
  int iterations{0};
  Scalar mean_residual{0.0};
  Scalar final_update_norm{0.0};
  Scalar elapsed_ms{0.0};
  SE3Pose T_world_imu;
  Eigen::Matrix<Scalar, 6, 6> information{
    Eigen::Matrix<Scalar, 6, 6>::Identity()};
};

class VendorLioEskf {
public:
  VendorLioEskf(const SensorParams& sensor_params, const LIOParams& lio_params);

  void initialize(const SE3Pose& pose,
                  const Eigen::Matrix<Scalar, 3, 1>& velocity,
                  const Eigen::Matrix<Scalar, 3, 1>& gyro_bias,
                  const Eigen::Matrix<Scalar, 3, 1>& accel_bias);

  /// Set the nominal state produced by IMU propagation, then propagate its
  /// covariance using the same IMU interval.
  void setPredictedNominal(const SE3Pose& pose,
                           const Eigen::Matrix<Scalar, 3, 1>& velocity);
  void propagateCovariance(const std::vector<ImuPacket>& measurements);

  VendorLioUpdateResult update(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_lidar,
    const std::shared_ptr<VoxelMap>& voxel_map);

  const VendorLioState& state() const { return state_; }

private:
  using ErrorVector = Eigen::Matrix<Scalar, VendorLioState::kErrorDim, 1>;
  using StateMatrix = VendorLioState::Covariance;

  void applyError(const ErrorVector& error);
  ErrorVector boxMinus(const VendorLioState& reference) const;
  Eigen::Matrix<Scalar, 3, 2> gravityTangentBasis() const;

  SensorParams sensor_params_;
  LIOParams lio_params_;
  VendorLioState state_;
  bool initialized_{false};
};

}  // namespace m20::lio
