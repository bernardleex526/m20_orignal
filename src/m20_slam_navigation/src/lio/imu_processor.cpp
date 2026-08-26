#include "m20_slam_navigation/lio/imu_processor.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <algorithm>
#include <cstring>

namespace m20::lio {

ImuProcessor::ImuProcessor(const SensorParams& sensor_params, const LIOParams& lio_params)
    : sensor_params_(sensor_params), lio_params_(lio_params) {}

void ImuProcessor::addMeasurement(const ImuPacket& imu) {
  if (imu_buffer_.size() >= kMaxBufferSize) {
    imu_buffer_.pop_front();
  }
  imu_buffer_.push_back(imu);
}

ImuProcessor::IntegrationResult ImuProcessor::integrate(
    const Timestamp& t_start, const Timestamp& t_end,
    const SE3Pose& pose_start,
    const Eigen::Matrix<Scalar, 3, 1>& vel_start,
    const Eigen::Matrix<Scalar, 3, 1>& ba,
    const Eigen::Matrix<Scalar, 3, 1>& bg) {

  IntegrationResult result;
  result.pose_end = pose_start;
  result.velocity_end = vel_start;

  // Collect IMU measurements in [t_start, t_end]
  std::vector<ImuPacket> measurements;
  {
    for (const auto& m : imu_buffer_) {
      if (m.stamp >= t_start && m.stamp <= t_end) {
        measurements.push_back(m);
      }
    }
  }

  if (measurements.empty()) {
    // No IMU data: return start pose
    result.trajectory.push_back({t_start, pose_start});
    result.gravity_direction = pose_start.q.conjugate()._transformVector(lio_params_.gravity);
    return result;
  }

  SE3Pose pose = pose_start;
  Eigen::Matrix<Scalar, 3, 1> vel = vel_start;
  result.trajectory.reserve(measurements.size() + 1);
  result.trajectory.push_back({t_start, pose});

  // Integrate using mid-point method on SE(3)
  for (std::size_t i = 0; i < measurements.size() - 1; ++i) {
    propagateState(measurements[i], measurements[i + 1], pose, vel, ba, bg);
    result.trajectory.push_back({measurements[i + 1].stamp, pose});
  }

  result.pose_end = pose;
  result.velocity_end = vel;
  // Gravity direction in body frame: R_wbᵀ · [0, 0, -g]
  result.gravity_direction = pose.q.conjugate()._transformVector(lio_params_.gravity);

  return result;
}

void ImuProcessor::propagateState(
    const ImuPacket& imu0, const ImuPacket& imu1,
    SE3Pose& pose, Eigen::Matrix<Scalar, 3, 1>& vel,
    const Eigen::Matrix<Scalar, 3, 1>& ba,
    const Eigen::Matrix<Scalar, 3, 1>& bg) {

  using namespace std::chrono;
  Scalar dt = static_cast<Scalar>(
      duration_cast<nanoseconds>(imu1.stamp - imu0.stamp).count()) * 1e-9;

  if (dt <= 0) return;

  // Mid-point: average acceleration and angular velocity
  Eigen::Matrix<Scalar, 3, 1> a0 = imu0.accel - ba;
  Eigen::Matrix<Scalar, 3, 1> a1 = imu1.accel - ba;
  Eigen::Matrix<Scalar, 3, 1> g0 = imu0.gyro - bg;
  Eigen::Matrix<Scalar, 3, 1> g1 = imu1.gyro - bg;

  Eigen::Matrix<Scalar, 3, 1> a_mid = (a0 + a1) * 0.5;
  Eigen::Matrix<Scalar, 3, 1> g_mid = (g0 + g1) * 0.5;

  // Rotation update: R_k+1 = R_k · Exp(ω_mid · dt)
  Eigen::Matrix<Scalar, 3, 1> delta_angle = g_mid * dt;
  Eigen::Quaternion<Scalar> dq = Eigen::Quaternion<Scalar>(math::so3_exp(delta_angle));
  pose.q = pose.q * dq;
  pose.q.normalize();

  // Velocity update: v_k+1 = v_k + (R_k · a_mid + g_world) · dt
  Eigen::Matrix<Scalar, 3, 1> a_world = pose.q._transformVector(a_mid);
  vel += (a_world + lio_params_.gravity) * dt;

  // Position update: p_k+1 = p_k + v_k·dt + ½·(R_k·a_mid + g)·dt²
  pose.t += vel * dt + 0.5 * (a_world + lio_params_.gravity) * dt * dt;
}

Eigen::Quaternion<Scalar> ImuProcessor::estimateInitialAttitude(
    const std::vector<ImuPacket>& buffer, std::size_t num_samples) {

  if (buffer.empty()) {
    return Eigen::Quaternion<Scalar>::Identity();
  }

  // Average acceleration over num_samples to get gravity direction estimate
  Eigen::Matrix<Scalar, 3, 1> avg_accel{0, 0, 0};
  std::size_t n = std::min(num_samples, buffer.size());

  for (std::size_t i = 0; i < n; ++i) {
    avg_accel += buffer[i].accel;
  }
  avg_accel /= static_cast<Scalar>(n);

  // Normalize to get gravity direction in body frame
  Scalar norm = avg_accel.norm();
  if (norm < 1e-6) {
    return Eigen::Quaternion<Scalar>::Identity();
  }
  avg_accel.normalize();

  // Compute rotation that aligns body-frame gravity with world-frame [0, 0, -g]
  // Using the shortest-arc quaternion between two vectors
  Eigen::Matrix<Scalar, 3, 1> g_body = avg_accel;
  Eigen::Matrix<Scalar, 3, 1> g_world(0, 0, -1);  // normalized gravity in world (z-up)

  // Rotation axis: cross product; angle: acos(dot)
  Eigen::Matrix<Scalar, 3, 1> axis = g_body.cross(g_world);
  Scalar axis_norm = axis.norm();
  Scalar dot = g_body.dot(g_world);

  if (axis_norm < 1e-10) {
    // Vectors are parallel or anti-parallel
    if (dot > 0) return Eigen::Quaternion<Scalar>::Identity();
    // Anti-parallel: 180° around any perpendicular axis
    Eigen::Matrix<Scalar, 3, 1> perp = std::abs(g_body.x()) < 0.9
        ? Eigen::Matrix<Scalar, 3, 1>(1, 0, 0)
        : Eigen::Matrix<Scalar, 3, 1>(0, 1, 0);
    return Eigen::Quaternion<Scalar>(
        Eigen::AngleAxis<Scalar>(math::kPI, perp.cross(g_body).normalized()));
  }

  axis /= axis_norm;
  Scalar angle = std::acos(math::clamp(dot, Scalar(-1), Scalar(1)));
  return Eigen::Quaternion<Scalar>(Eigen::AngleAxis<Scalar>(angle, axis));
}

std::vector<ImuPacket> ImuProcessor::getMeasurementsBetween(
    const Timestamp& from, const Timestamp& to) const {
  std::vector<ImuPacket> result;
  for (const auto& m : imu_buffer_) {
    if (m.stamp >= from && m.stamp <= to) {
      result.push_back(m);
    }
  }
  return result;
}

void ImuProcessor::reset() {
  imu_buffer_.clear();
}

}  // namespace m20::lio