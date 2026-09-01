#include "m20_slam_navigation/localization/eskf.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <cmath>

namespace m20::localization {

namespace {

Eigen::Matrix<Scalar, 6, 1> poseInnovation(
    const ESKFState& state, const SE3Pose& measured) {
  Eigen::Matrix<Scalar, 6, 1> innovation;
  innovation.head<3>() = measured.t - state.p;
  const Eigen::Quaternion<Scalar> delta_q = state.q.conjugate() * measured.q;
  innovation.tail<3>() = math::so3_log(delta_q.normalized().toRotationMatrix());
  return innovation;
}

void applyPoseUpdate(
    ESKFState& state,
    Eigen::Matrix<Scalar, 15, 1>& error_state,
    const Eigen::Matrix<Scalar, 6, 1>& innovation,
    const Eigen::Matrix<Scalar, 6, 6>& covariance) {
  Eigen::Matrix<Scalar, 6, 15> H = Eigen::Matrix<Scalar, 6, 15>::Zero();
  H.block<3, 3>(0, 0).setIdentity();
  H.block<3, 3>(3, 3).setIdentity();

  const Eigen::Matrix<Scalar, 6, 6> S = H * state.P * H.transpose() + covariance;
  const Eigen::Matrix<Scalar, 15, 6> K =
      state.P * H.transpose() * S.ldlt().solve(Eigen::Matrix<Scalar, 6, 6>::Identity());
  error_state += K * innovation;

  const Eigen::Matrix<Scalar, 15, 15> I =
      Eigen::Matrix<Scalar, 15, 15>::Identity();
  const Eigen::Matrix<Scalar, 15, 15> IKH = I - K * H;
  state.P = IKH * state.P * IKH.transpose() + K * covariance * K.transpose();
}

}  // namespace

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
  const Eigen::Matrix<Scalar, 3, 1> gravity_world(
      0.0, 0.0, -std::abs(params_.imu_gravity));
  const Eigen::Matrix<Scalar, 3, 1> acceleration_world = a_world + gravity_world;

  // Integrate position with the velocity at the beginning of the interval.
  state_.p += state_.v * dt + 0.5 * acceleration_world * dt * dt;
  state_.v += acceleration_world * dt;

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

  applyPoseUpdate(state_, error_state_, poseInnovation(state_, T_measured), covariance);
}

void ESKF::updateOdometry(const SE3Pose& T_measured,
                           const Eigen::Matrix<Scalar, 6, 6>& covariance) {

  if (!initialized_) return;

  applyPoseUpdate(state_, error_state_, poseInnovation(state_, T_measured), covariance);
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
