#include "m20_slam_navigation/common/math_utils.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/localization/eskf.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace m20::localization {
namespace {

LocalizationParams testParams() {
  LocalizationParams params;
  params.imu_gravity = 9.80511;
  return params;
}

TEST(LocalizationEskf, GravityCompensatedStationaryStateDoesNotDrift) {
  ESKF eskf(testParams());
  eskf.initialize(SE3Pose::Identity());

  for (int i = 0; i < 100; ++i) {
    eskf.predict({0.0, 0.0, 9.80511}, {0.0, 0.0, 0.0}, 0.01);
  }

  EXPECT_NEAR(eskf.getPose().t.norm(), 0.0, 1e-9);
  EXPECT_NEAR(eskf.getVelocity().norm(), 0.0, 1e-9);
}

TEST(LocalizationEskf, ConstantAccelerationUsesBeginningOfIntervalVelocity) {
  ESKF eskf(testParams());
  eskf.initialize(SE3Pose::Identity());

  for (int i = 0; i < 100; ++i) {
    eskf.predict({1.0, 0.0, 9.80511}, {0.0, 0.0, 0.0}, 0.01);
  }

  EXPECT_NEAR(eskf.getVelocity().x(), 1.0, 1e-9);
  EXPECT_NEAR(eskf.getPose().t.x(), 0.5, 1e-9);
}

TEST(LocalizationEskf, PositionObservationDoesNotBecomeRotationError) {
  ESKF eskf(testParams());
  eskf.initialize(SE3Pose::Identity());

  SE3Pose measured;
  measured.t.x() = 1.0;
  const auto covariance = Eigen::Matrix<Scalar, 6, 6>::Identity() * 1e-4;
  eskf.updateOdometry(measured, covariance);
  eskf.injectErrorAndReset();

  EXPECT_GT(eskf.getPose().t.x(), 0.9);
  EXPECT_NEAR(math::quaternion_to_yaw(eskf.getPose().q), 0.0, 1e-9);
}

TEST(LocalizationEskf, YawObservationDoesNotTranslatePosition) {
  ESKF eskf(testParams());
  eskf.initialize(SE3Pose::Identity());

  SE3Pose measured;
  measured.q = math::yaw_to_quaternion(0.2);
  const auto covariance = Eigen::Matrix<Scalar, 6, 6>::Identity() * 1e-4;
  eskf.updateNDT(measured, covariance);
  eskf.injectErrorAndReset();

  EXPECT_NEAR(eskf.getPose().t.norm(), 0.0, 1e-9);
  EXPECT_GT(math::quaternion_to_yaw(eskf.getPose().q), 0.15);
}

}  // namespace
}  // namespace m20::localization
