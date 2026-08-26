#include "m20_slam_navigation/localization/eskf.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <cmath>

namespace m20::localization {

ESKF::ESKF(const LocalizationParams& params) : params_(params) {}

void ESKF::initialize(const SE3Pose& initial_pose,
                       const Eigen::Matrix<Scalar, 3, 1>& initial_vel) {
  state_.p = initial_pose.t;
  state_.q = initial_pose.q;
  state_.v = initial_vel;
  state_.ba.setZero();
  state_.bg.setZero();

  // Initial covariance: moderate uncertainty
  state_.P = Eigen::Matrix<Scalar, 15, 15>::Identity() * 0.1;
  // Higher uncertainty in position (allow convergence)
  state_.P.block<3, 3>(0, 0) *= 10.0;   // δp
  state_.P.block<3, 3>(3, 3) *= 0.5;    // δθ
  state_.P.block<3, 3>(6, 6) *= 1.0;    // δv
  state_.P.block<3, 3>(9, 9) *= 0.01;   // δba
  state_.P.block<3, 3>(12, 12) *= 0.01; // δbg

  error_state_.setZero();
  initialized_ = true;
}

void ESKF::predict(const Eigen::Matrix<Scalar, 3, 1>& accel,
                    const Eigen::Matrix<Scalar, 3, 1>& gyro,
                    Scalar dt) {

  if (!initialized_ || dt <= 0) return;

  // ---- Nominal state propagation (non-linear) ----
  // Remove bias
  Eigen::Matrix<Scalar, 3, 1> a = accel - state_.ba;
  Eigen::Matrix<Scalar, 3, 1> g = gyro - state_.bg;

  // Rotation: q ← q ⊗ q{ω·dt}
  Eigen::Matrix<Scalar, 3, 1> delta_angle = g * dt;
  Eigen::Quaternion<Scalar> dq = Eigen::Quaternion<Scalar>(math::so3_exp(delta_angle));
  state_.q = state_.q * dq;
  state_.q.normalize();

  // Velocity: v ← v + (R·a + g_world)·dt
  Eigen::Matrix<Scalar, 3, 1> a_world = state_.q._transformVector(a);
  state_.v += (a_world + ESKFState::GRAVITY) * dt;

  // Position: p ← p + v·dt + ½(R·a + g_world)·dt²
  state_.p += state_.v * dt + 0.5 * (a_world + ESKFState::GRAVITY) * dt * dt;

  // ---- Error state propagation (linear) ----
  // Compute transition matrix A (15×15)
  Eigen::Matrix<Scalar, 15, 15> A;
  computeTransitionMatrix(accel, gyro, dt, A);

  // Process noise covariance Q
  Eigen::Matrix<Scalar, 15, 15> Q = Eigen::Matrix<Scalar, 15, 15>::Zero();
  // Noise on acceleration (3-DOF)
  Q.block<3, 3>(6, 6) = Eigen::Matrix<Scalar, 3, 3>::Identity() *
                         params_.eskf_accel_noise * params_.eskf_accel_noise * dt * dt;
  // Noise on gyro
  Q.block<3, 3>(3, 3) = Eigen::Matrix<Scalar, 3, 3>::Identity() *
                         params_.eskf_gyro_noise * params_.eskf_gyro_noise * dt * dt;
  // Bias random walks
  Q.block<3, 3>(9, 9) = Eigen::Matrix<Scalar, 3, 3>::Identity() *
                         params_.eskf_accel_bias_noise * params_.eskf_accel_bias_noise * dt;
  Q.block<3, 3>(12, 12) = Eigen::Matrix<Scalar, 3, 3>::Identity() *
                          params_.eskf_gyro_bias_noise * params_.eskf_gyro_bias_noise * dt;

  // Error-state covariance propagation: P = A·P·Aᵀ + Q
  state_.P = A * state_.P * A.transpose() + Q;
}

void ESKF::updateNDT(const SE3Pose& T_measured,
                      const Eigen::Matrix<Scalar, 6, 6>& covariance) {

  if (!initialized_) return;

  // ---- Compute predicted measurement h(x_nom) ----
  // For pose measurement, the residual is the SE(3) difference in local coordinates:
  //   z = h(x) ⊞ v = log(T_nom⁻¹ ∘ T_measured)  (6×1 twist)
  SE3Pose T_nom(state_.q, state_.p);
  SE3Pose delta_T = T_nom.inverse() * T_measured;
  Eigen::Matrix<Scalar, 6, 1> innovation = delta_T.log();

  // ---- Observation Jacobian H (6×15) ----
  // For NDT pose observation: H = [I₆ | 0₆ₓ₉]
  Eigen::Matrix<Scalar, 6, 15> H = Eigen::Matrix<Scalar, 6, 15>::Zero();
  H.block<6, 6>(0, 0) = Eigen::Matrix<Scalar, 6, 6>::Identity();

  // ---- Measurement noise R ----
  Eigen::Matrix<Scalar, 6, 6> R = covariance;

  // ---- Kalman update ----
  // S = H·P·Hᵀ + R
  Eigen::Matrix<Scalar, 6, 6> S = H * state_.P * H.transpose() + R;

  // K = P·Hᵀ·S⁻¹
  Eigen::Matrix<Scalar, 15, 6> K = state_.P * H.transpose() * S.inverse();

  // δx = K·(z − h(x))
  Eigen::Matrix<Scalar, 15, 1> dx = K * innovation;

  // ---- Update error state ----
  error_state_ += dx;

  // P = (I − K·H)·P
  state_.P = (Eigen::Matrix<Scalar, 15, 15>::Identity() - K * H) * state_.P;
}

void ESKF::updateOdometry(const SE3Pose& T_measured,
                           const Eigen::Matrix<Scalar, 6, 6>& covariance) {

  if (!initialized_) return;

  // Same formulation as NDT update but with different noise
  SE3Pose T_nom(state_.q, state_.p);
  SE3Pose delta_T = T_nom.inverse() * T_measured;
  Eigen::Matrix<Scalar, 6, 1> innovation = delta_T.log();

  Eigen::Matrix<Scalar, 6, 15> H = Eigen::Matrix<Scalar, 6, 15>::Zero();
  H.block<6, 6>(0, 0) = Eigen::Matrix<Scalar, 6, 6>::Identity();

  Eigen::Matrix<Scalar, 6, 6> S = H * state_.P * H.transpose() + covariance;
  Eigen::Matrix<Scalar, 15, 6> K = state_.P * H.transpose() * S.inverse();
  Eigen::Matrix<Scalar, 15, 1> dx = K * innovation;

  error_state_ += dx;
  state_.P = (Eigen::Matrix<Scalar, 15, 15>::Identity() - K * H) * state_.P;
}

void ESKF::injectErrorAndReset() {
  if (!initialized_) return;

  // Inject error into nominal state
  // δp → p += δp
  state_.p += error_state_.segment<3>(0);

  // δθ → q = q ⊗ exp(δθ)
  Eigen::Matrix<Scalar, 3, 1> dtheta = error_state_.segment<3>(3);
  Eigen::Quaternion<Scalar> dq = Eigen::Quaternion<Scalar>(math::so3_exp(dtheta));
  state_.q = state_.q * dq;
  state_.q.normalize();

  // δv → v += δv
  state_.v += error_state_.segment<3>(6);

  // δba → ba += δba, δbg → bg += δbg
  state_.ba += error_state_.segment<3>(9);
  state_.bg += error_state_.segment<3>(12);

  // Reset error state to zero
  error_state_.setZero();

  // Reset covariance: project P through the error reset Jacobian G
  // G ≈ I₁₅ (to first order for small δθ)
  // For exact reset: G = I − [½δθ]× on the rotation block
  Eigen::Matrix<Scalar, 15, 15> G = Eigen::Matrix<Scalar, 15, 15>::Identity();
  G.block<3, 3>(3, 3) -= 0.5 * math::skew(dtheta);
  state_.P = G * state_.P * G.transpose();

  // Enforce symmetry
  state_.P = 0.5 * (state_.P + state_.P.transpose());
}

SE3Pose ESKF::getPose() const {
  return SE3Pose{state_.q, state_.p};
}

void ESKF::computeTransitionMatrix(
    const Eigen::Matrix<Scalar, 3, 1>& accel,
    const Eigen::Matrix<Scalar, 3, 1>& gyro,
    Scalar dt,
    Eigen::Matrix<Scalar, 15, 15>& A) const {

  // Continuous-time error-state dynamics:
  //   δṗ = δv
  //   δθ̇ = −[ω_m − b_g]× · δθ − δb_g
  //   δv̇ = −R·[a_m − b_a]× · δθ − R·δb_a
  //   δḃa = white noise (zero in transition)
  //   δḃg = white noise (zero in transition)

  Eigen::Matrix<Scalar, 3, 3> R = state_.q.toRotationMatrix();
  Eigen::Matrix<Scalar, 3, 1> a_corrected = accel - state_.ba;
  Eigen::Matrix<Scalar, 3, 1> g_corrected = gyro - state_.bg;

  A = Eigen::Matrix<Scalar, 15, 15>::Identity();  // Start with I for discrete-time

  // First-order discrete approximation: A ≈ I + F·dt
  // where F is the continuous-time transition matrix

  // δṗ → δv:  A_pv = I·dt
  A.block<3, 3>(0, 6) = Eigen::Matrix<Scalar, 3, 3>::Identity() * dt;

  // δθ̇ → −[ω]×·δθ:  A_theta_theta = I − [ω]×·dt
  A.block<3, 3>(3, 3) -= math::skew(g_corrected) * dt;

  // δθ̇ → −δb_g:  A_theta_bg = −I·dt
  A.block<3, 3>(3, 12) = -Eigen::Matrix<Scalar, 3, 3>::Identity() * dt;

  // δv̇ → −R·[a]×·δθ:  A_v_theta = −R·[a]×·dt
  A.block<3, 3>(6, 3) = -R * math::skew(a_corrected) * dt;

  // δv̇ → −R·δb_a:  A_v_ba = −R·dt
  A.block<3, 3>(6, 9) = -R * dt;

  // Position → velocity already covered
  // Position → rotation (indirectly through velocity integration):
  A.block<3, 3>(0, 3) = -0.5 * R * math::skew(a_corrected) * dt * dt;
}

}  // namespace m20::localization