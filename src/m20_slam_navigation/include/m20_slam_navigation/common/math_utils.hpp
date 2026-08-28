#pragma once
/**
 * @file math_utils.hpp
 * @brief Lie algebra utilities for SE(3)/SO(3), transforms, and numerical helpers.
 *
 * Notation:
 *   - se(3) twist: ξ = [ω; v] ∈ ℝ⁶  (angular, linear)
 *   - Hat operator ^: ℝ⁶ → se(3) 4×4 matrix
 *   - Vee operator v: se(3) → ℝ⁶
 *   - SO(3) → so(3) via Rodrigues / log map
 */

#include "m20_slam_navigation/common/types.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <cmath>

namespace m20::math {

// =============================================================================
// Constants
// =============================================================================
inline constexpr Scalar kPI   = 3.14159265358979323846;
inline constexpr Scalar k2PI  = 2.0 * kPI;
inline constexpr Scalar kDeg2Rad = kPI / 180.0;
inline constexpr Scalar kRad2Deg = 180.0 / kPI;

// =============================================================================
// SO(3) ↔ so(3) (rotation matrix / quaternion ↔ axis-angle)
// =============================================================================

/**
 * @brief Logarithmic map from SO(3) → so(3) (axis-angle representation).
 *
 * Given rotation matrix R ∈ SO(3), compute ω = log(R)∨ where
 *   θ = acos((trace(R) − 1) / 2)
 *   ω = θ / (2 sin θ) · (R − Rᵀ)∨
 *
 * @return 3×1 axis-angle vector ω (magnitude = rotation angle [rad])
 */
Eigen::Matrix<Scalar, 3, 1> so3_log(const Eigen::Matrix<Scalar, 3, 3>& R);

/**
 * @brief Exponential map from so(3) → SO(3).
 *
 * Rodrigues formula: R = I + sin(θ)/θ · [ω]× + (1 − cos(θ))/θ² · [ω]×²
 * where θ = ‖ω‖.
 */
Eigen::Matrix<Scalar, 3, 3> so3_exp(const Eigen::Matrix<Scalar, 3, 1>& omega);

/**
 * @brief Right Jacobian of SO(3): Jr(ω) = I − (1−cos θ)/θ²·[ω]× + (θ−sin θ)/θ³·[ω]×²
 *
 * Used for uncertainty propagation in ESKF and manifold optimization.
 */
Eigen::Matrix<Scalar, 3, 3> so3_right_jacobian(const Eigen::Matrix<Scalar, 3, 1>& omega);

/**
 * @brief Inverse of SO(3) right Jacobian.
 */
Eigen::Matrix<Scalar, 3, 3> so3_right_jacobian_inv(const Eigen::Matrix<Scalar, 3, 1>& omega);

// =============================================================================
// SE(3) ↔ se(3)
// =============================================================================

/**
 * @brief Hat operator: ℝ⁶ → se(3) 4×4 matrix.
 *
 * ξ = [ω; v]  →  ξ^ = [ [ω]×   v ]
 *                      [ 0     0 ]
 */
Eigen::Matrix<Scalar, 4, 4> se3_hat(const Eigen::Matrix<Scalar, 6, 1>& xi);

/**
 * @brief Adjoint transformation Ad_T : se(3) → se(3).
 *
 * Ad_T(ξ) = [ R · ω ]
 *           [ [t]× · R · ω + R · v ]
 *
 * Or in 6×6 matrix form:
 *   Ad_T = [ R      0 ]
 *          [ [t]×·R  R ]
 */
Eigen::Matrix<Scalar, 6, 6> se3_adjoint(const SE3Pose& T);

// =============================================================================
// Skew-symmetric matrix
// =============================================================================

/// [v]× ∈ so(3) for v ∈ ℝ³
inline Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 1>& v) {
  Eigen::Matrix<Scalar, 3, 3> S;
  S << 0,    -v.z(),  v.y(),
       v.z(), 0,     -v.x(),
      -v.y(), v.x(),  0;
  return S;
}

// =============================================================================
// Quaternion helpers
// =============================================================================

/// Normalize angle to [-π, π]
inline Scalar normalize_angle(Scalar rad) {
  rad = std::fmod(rad, k2PI);
  if (rad > kPI)  rad -= k2PI;
  if (rad < -kPI) rad += k2PI;
  return rad;
}

/// Extract yaw from quaternion: atan2(2(wz+xy), 1-2(y²+z²))
inline Scalar quaternion_to_yaw(const Eigen::Quaternion<Scalar>& q) {
  Scalar siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  Scalar cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

/// Quaternion from yaw only (zero roll/pitch)
inline Eigen::Quaternion<Scalar> yaw_to_quaternion(Scalar yaw) {
  Scalar half = yaw * 0.5;
  return Eigen::Quaternion<Scalar>(std::cos(half), 0.0, 0.0, std::sin(half));
}

// =============================================================================
// Numerical helpers
// =============================================================================

/// Safe sqrt for covariance inflation
inline Scalar safe_sqrt(Scalar x) { return std::sqrt(std::max(x, Scalar(0))); }

/// Clamp value to [lo, hi]
template <typename T>
inline T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

/// Check if matrix is well-conditioned for inversion
template <typename Derived>
bool is_well_conditioned(const Eigen::MatrixBase<Derived>& M, Scalar max_cond = 1e8) {
  Eigen::JacobiSVD<typename Derived::PlainObject> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& S = svd.singularValues();
  if (S(0) < 1e-12) return false;
  return (S(0) / S(S.size() - 1)) < max_cond;
}

/// Compute Mahalanobis distance: d = (x−μ)ᵀ Σ⁻¹ (x−μ)
inline Scalar mahalanobis(const Eigen::Matrix<Scalar, 3, 1>& x,
                          const Eigen::Matrix<Scalar, 3, 1>& mu,
                          const Eigen::Matrix<Scalar, 3, 3>& sigma) {
  Eigen::Matrix<Scalar, 3, 1> d = x - mu;
  // Use LDLT for symmetric positive semi-definite
  return d.transpose() * sigma.ldlt().solve(d);
}

}  // namespace m20::math
