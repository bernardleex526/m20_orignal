#include "m20_slam_navigation/common/math_utils.hpp"

#include <Eigen/Geometry>

namespace m20::math {

// =============================================================================
// SO(3) Logarithmic Map: R ↦ ω (axis-angle)
// =============================================================================
Eigen::Matrix<Scalar, 3, 1> so3_log(const Eigen::Matrix<Scalar, 3, 3>& R) {
  // θ = acos((tr(R) − 1) / 2), clamped for numerical stability
  Scalar cos_theta = (R.trace() - 1.0) * 0.5;
  cos_theta = clamp(cos_theta, Scalar(-1.0), Scalar(1.0));
  Scalar theta = std::acos(cos_theta);

  Eigen::Matrix<Scalar, 3, 1> omega;
  if (std::abs(theta) < 1e-10) {
    // Small angle: ω ≈ (R − Rᵀ)∨ / 2
    omega << R(2, 1) - R(1, 2),
             R(0, 2) - R(2, 0),
             R(1, 0) - R(0, 1);
    omega *= 0.5;
  } else {
    Scalar sin_theta = std::sin(theta);
    Scalar factor = theta / (2.0 * sin_theta);
    omega << factor * (R(2, 1) - R(1, 2)),
             factor * (R(0, 2) - R(2, 0)),
             factor * (R(1, 0) - R(0, 1));
  }
  return omega;
}

// =============================================================================
// SO(3) Exponential Map: ω ↦ R (Rodrigues formula)
// =============================================================================
Eigen::Matrix<Scalar, 3, 3> so3_exp(const Eigen::Matrix<Scalar, 3, 1>& omega) {
  Scalar theta = omega.norm();
  Eigen::Matrix<Scalar, 3, 3> S = skew(omega);

  if (theta < 1e-10) {
    // R ≈ I + [ω]× + ½[ω]×²
    return Eigen::Matrix<Scalar, 3, 3>::Identity() + S + 0.5 * S * S;
  }

  Scalar sin_theta = std::sin(theta);
  Scalar one_minus_cos = 1.0 - std::cos(theta);
  Scalar factor1 = sin_theta / theta;
  Scalar factor2 = one_minus_cos / (theta * theta);

  return Eigen::Matrix<Scalar, 3, 3>::Identity()
       + factor1 * S
       + factor2 * S * S;
}

// =============================================================================
// SO(3) Right Jacobian: Jr(ω) = I − (1−cos θ)/θ² · [ω]× + (θ−sin θ)/θ³ · [ω]×²
// =============================================================================
Eigen::Matrix<Scalar, 3, 3> so3_right_jacobian(const Eigen::Matrix<Scalar, 3, 1>& omega) {
  Scalar theta = omega.norm();
  Eigen::Matrix<Scalar, 3, 3> S = skew(omega);

  if (theta < 1e-10) {
    return Eigen::Matrix<Scalar, 3, 3>::Identity() - 0.5 * S + (1.0 / 6.0) * S * S;
  }

  Scalar sin_theta = std::sin(theta);
  Scalar cos_theta = std::cos(theta);
  Scalar theta2 = theta * theta;
  Scalar theta3 = theta2 * theta;

  Scalar factor1 = (1.0 - cos_theta) / theta2;
  Scalar factor2 = (theta - sin_theta) / theta3;

  return Eigen::Matrix<Scalar, 3, 3>::Identity()
       - factor1 * S
       + factor2 * S * S;
}

// =============================================================================
// SO(3) Inverse Right Jacobian
// =============================================================================
Eigen::Matrix<Scalar, 3, 3> so3_right_jacobian_inv(const Eigen::Matrix<Scalar, 3, 1>& omega) {
  Scalar theta = omega.norm();
  Eigen::Matrix<Scalar, 3, 3> S = skew(omega);

  if (theta < 1e-10) {
    return Eigen::Matrix<Scalar, 3, 3>::Identity() + 0.5 * S + (1.0 / 12.0) * S * S;
  }

  Scalar half_theta = 0.5 * theta;
  Scalar cot_half = std::cos(half_theta) / std::sin(half_theta);  // cot(θ/2)
  Scalar factor = 0.5 * theta * cot_half;

  Scalar factor1 = 0.5;
  Scalar factor2 = (1.0 - factor) / (theta * theta);

  return Eigen::Matrix<Scalar, 3, 3>::Identity()
       + factor1 * S
       + factor2 * S * S;
}

// =============================================================================
// SE(3) Hat Operator: ξ ↦ ξ^ ∈ se(3)
// =============================================================================
Eigen::Matrix<Scalar, 4, 4> se3_hat(const Eigen::Matrix<Scalar, 6, 1>& xi) {
  Eigen::Matrix<Scalar, 4, 4> Xi = Eigen::Matrix<Scalar, 4, 4>::Zero();
  Xi.template block<3, 3>(0, 0) = skew(xi.head<3>());
  Xi.template block<3, 1>(0, 3) = xi.tail<3>();
  return Xi;
}

// =============================================================================
// SE(3) Adjoint: Ad_T(ξ)
// =============================================================================
Eigen::Matrix<Scalar, 6, 6> se3_adjoint(const SE3Pose& T) {
  Eigen::Matrix<Scalar, 3, 3> R = T.q.toRotationMatrix();
  Eigen::Matrix<Scalar, 3, 3> t_skew = skew(T.t);

  Eigen::Matrix<Scalar, 6, 6> Ad = Eigen::Matrix<Scalar, 6, 6>::Zero();
  Ad.template block<3, 3>(0, 0) = R;
  Ad.template block<3, 3>(3, 0) = t_skew * R;
  Ad.template block<3, 3>(3, 3) = R;
  return Ad;
}

}  // namespace m20::math

// =============================================================================
// SE3Pose member functions
// =============================================================================
namespace m20 {

Eigen::Matrix<Scalar, 6, 1> SE3Pose::log() const {
  Eigen::Matrix<Scalar, 6, 1> xi;
  xi.head<3>() = math::so3_log(q.toRotationMatrix());

  // Translation component: v = Jr⁻¹(ω) · t
  Eigen::Matrix<Scalar, 3, 3> Jr_inv = math::so3_right_jacobian_inv(xi.head<3>());
  xi.tail<3>() = Jr_inv * t;

  return xi;
}

SE3Pose SE3Pose::exp(const Eigen::Matrix<Scalar, 6, 1>& xi) {
  SE3Pose out;
  Eigen::Matrix<Scalar, 3, 3> R = math::so3_exp(xi.head<3>());
  out.q = Eigen::Quaternion<Scalar>(R);

  Eigen::Matrix<Scalar, 3, 3> Jr = math::so3_right_jacobian(xi.head<3>());
  out.t = Jr * xi.tail<3>();

  return out;
}

}  // namespace m20