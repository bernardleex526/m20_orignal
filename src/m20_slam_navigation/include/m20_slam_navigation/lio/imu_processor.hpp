#pragma once
/**
 * @file imu_processor.hpp
 * @brief IMU pre-integration and forward propagation for LIO front-end.
 *
 * Integrates high-rate IMU measurements (200Hz+) between LiDAR frames to:
 *  1. Provide pose prior for scan-to-map registration.
 *  2. Deskew point clouds (motion compensation) using per-point timestamps.
 *  3. Estimate gravity direction for roll/pitch initialization.
 *
 * Integration uses the mid-point method on SE(3) manifold:
 *   R_k+1 = R_k · Exp((ω_k − b_g) · Δt)
 *   v_k+1 = v_k + (R_k · (a_k − b_a) + g) · Δt
 *   p_k+1 = p_k + v_k · Δt + ½ (R_k · (a_k − b_a) + g) · Δt²
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Dense>

#include <deque>
#include <memory>
#include <mutex>
#include <vector>

namespace m20::lio {

class ImuProcessor {
public:
  explicit ImuProcessor(const SensorParams& sensor_params, const LIOParams& lio_params);

  /// Feed raw IMU measurement into the buffer
  void addMeasurement(const ImuPacket& imu);

  /// Integrate IMU from t_start to t_end, starting from initial pose.
  /// Returns the propagated pose at t_end plus intermediate poses for deskewing.
  struct IntegrationResult {
    SE3Pose                              pose_end;
    std::vector<std::pair<Timestamp, SE3Pose>> trajectory;  ///< time-stamped poses for deskewing
    Eigen::Matrix<Scalar, 3, 1>          velocity_end;
    Eigen::Matrix<Scalar, 3, 1>          gravity_direction; ///< estimated gravity vector in body frame
  };

  IntegrationResult integrate(const Timestamp& t_start, const Timestamp& t_end,
                              const SE3Pose& pose_start,
                              const Eigen::Matrix<Scalar, 3, 1>& vel_start,
                              const Eigen::Matrix<Scalar, 3, 1>& ba,
                              const Eigen::Matrix<Scalar, 3, 1>& bg);

  /// Estimate initial roll/pitch from stationary IMU data (gravity alignment)
  /// Returns quaternion that rotates gravity vector to align with [0, 0, -g]
  Eigen::Quaternion<Scalar> estimateInitialAttitude(const std::vector<ImuPacket>& buffer,
                                                     std::size_t num_samples = 100);

  /// Reset internal buffer
  void reset();

  /// Get buffered IMU data between two timestamps
  std::vector<ImuPacket> getMeasurementsBetween(const Timestamp& from, const Timestamp& to) const;

  /// Get all buffered measurements no later than the requested timestamp.
  /// Used by the vendor-compatible cumulative 200-sample initialization.
  std::vector<ImuPacket> getMeasurementsUpTo(const Timestamp& to) const;

private:
  /// Propagate state by one IMU step (mid-point integration on manifold)
  void propagateState(const ImuPacket& imu0, const ImuPacket& imu1,
                      SE3Pose& pose, Eigen::Matrix<Scalar, 3, 1>& vel,
                      const Eigen::Matrix<Scalar, 3, 1>& ba,
                      const Eigen::Matrix<Scalar, 3, 1>& bg);

  SensorParams    sensor_params_;
  LIOParams       lio_params_;
  std::deque<ImuPacket> imu_buffer_;
  mutable std::mutex imu_mutex_;
  static constexpr std::size_t kMaxBufferSize = 4096;
};

}  // namespace m20::lio
