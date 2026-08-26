#pragma once
/**
 * @file eskf.hpp
 * @brief Error-State Kalman Filter for tightly coupled localization.
 *
 * Fuses three sensor sources:
 *   1. IMU (high-rate, 200Hz):  forward propagation of nominal + error state.
 *   2. Foot Odometry (50-100Hz): position/velocity observation.
 *   3. NDT scan matching (low-rate, 1-5Hz): absolute pose observation.
 *
 * The ESKF separates nominal state (large, non-linear) from error state
 * (small, linearizable), avoiding gimbal lock and singularities:
 *
 *   Nominal state x = [p, q, v, ba, bg]  (16-dim)
 *   Error state   δx = [δp, δθ, δv, δba, δbg]  (15-dim, δθ ∈ so(3))
 *
 * Prediction (IMU):
 *   ẋ = f(x, u)          (nominal, non-linear)
 *   δẋ = A·δx + B·w      (error-state, linear)
 *   P = A·P·Aᵀ + B·Q·Bᵀ
 *
 * Update (NDT / Odom):
 *   K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
 *   δx = K·(z − h(x))
 *   P = (I − K·H)·P
 *   x = x ⊕ δx           (error reset + injection)
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Dense>

#include <functional>
#include <memory>

namespace m20::localization {

/// ESKF observation callback: provides predicted measurement and Jacobian
template <int DimZ>
struct ESKFObservation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<Scalar, DimZ, 1>          z;          ///< measurement vector
  Eigen::Matrix<Scalar, DimZ, 1>          z_pred;     ///< predicted measurement h(x_nom)
  Eigen::Matrix<Scalar, DimZ, 15>         H;          ///< Jacobian ∂h/∂δx at δx=0
  Eigen::Matrix<Scalar, DimZ, DimZ>       R;          ///< measurement noise covariance
};

class ESKF {
public:
  ESKF(const LocalizationParams& params);

  /// Initialize state
  void initialize(const SE3Pose& initial_pose,
                  const Eigen::Matrix<Scalar, 3, 1>& initial_vel = {0, 0, 0});

  // ---- Prediction ----

  /**
   * @brief Propagate state with IMU measurement.
   *
   * @param accel  Linear acceleration (body frame) [m/s²]
   * @param gyro   Angular velocity (body frame) [rad/s]
   * @param dt     Time step [s]
   */
  void predict(const Eigen::Matrix<Scalar, 3, 1>& accel,
               const Eigen::Matrix<Scalar, 3, 1>& gyro,
               Scalar dt);

  // ---- Updates ----

  /**
   * @brief NDT pose update (6-DOF observation).
   *
   * @param T_measured   Measured SE(3) pose from NDT
   * @param covariance   6×6 measurement covariance
   */
  void updateNDT(const SE3Pose& T_measured,
                 const Eigen::Matrix<Scalar, 6, 6>& covariance);

  /**
   * @brief Foot odometry update (6-DOF observation).
   *
   * @param T_measured   Measured SE(3) pose from leg kinematics
   * @param covariance   6×6 measurement covariance
   */
  void updateOdometry(const SE3Pose& T_measured,
                      const Eigen::Matrix<Scalar, 6, 6>& covariance);

  /**
   * @brief Generic update: inject error δx into nominal state and reset.
   *
   * After all updates, this performs x_nom ← x_nom ⊕ δx and resets P.
   */
  void injectErrorAndReset();

  // ---- Accessors ----

  SE3Pose getPose() const;
  Eigen::Matrix<Scalar, 3, 1> getVelocity() const { return state_.v; }
  Eigen::Matrix<Scalar, 3, 1> getAccelBias() const { return state_.ba; }
  Eigen::Matrix<Scalar, 3, 1> getGyroBias() const { return state_.bg; }
  Eigen::Matrix<Scalar, 15, 15> getCovariance() const { return state_.P; }

private:
  /// Compute the 15×15 transition matrix A for IMU propagation
  void computeTransitionMatrix(const Eigen::Matrix<Scalar, 3, 1>& accel,
                               const Eigen::Matrix<Scalar, 3, 1>& gyro,
                               Scalar dt,
                               Eigen::Matrix<Scalar, 15, 15>& A) const;

  LocalizationParams params_;
  ESKFState          state_;
  Eigen::Matrix<Scalar, 15, 1> error_state_{Eigen::Matrix<Scalar, 15, 1>::Zero()};
  bool               initialized_{false};
};

}  // namespace m20::localization