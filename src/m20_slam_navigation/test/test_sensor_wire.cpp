#include "m20_slam_navigation/ros/pointcloud_wire.hpp"

#include <gtest/gtest.h>

TEST(SensorWire, RoundTripsVendorImu)
{
  m20::ros::DrddsImu input;
  input.stamp_sec = 123;
  input.stamp_nanosec = 456789U;
  input.frame_id = "imu_link";
  input.orientation = {0.1, 0.2, 0.3, 0.9};
  input.orientation_covariance[0] = 1.0;
  input.angular_velocity = {1.1, 2.2, 3.3};
  input.angular_velocity_covariance[4] = 2.0;
  input.linear_acceleration = {4.4, 5.5, 6.6};
  input.linear_acceleration_covariance[8] = 3.0;

  const auto bytes = m20::ros::serializeImu(input);
  m20::ros::DrddsImu output;
  std::string error;
  ASSERT_TRUE(m20::ros::deserializeImu(bytes, output, error)) << error;
  EXPECT_EQ(output.stamp_sec, input.stamp_sec);
  EXPECT_EQ(output.stamp_nanosec, input.stamp_nanosec);
  EXPECT_EQ(output.frame_id, input.frame_id);
  EXPECT_DOUBLE_EQ(output.orientation.w, input.orientation.w);
  EXPECT_DOUBLE_EQ(output.angular_velocity.y, input.angular_velocity.y);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.z, input.linear_acceleration.z);
  EXPECT_DOUBLE_EQ(output.orientation_covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(output.angular_velocity_covariance[4], 2.0);
  EXPECT_DOUBLE_EQ(output.linear_acceleration_covariance[8], 3.0);
}

TEST(SensorWire, RejectsTruncatedVendorImu)
{
  m20::ros::DrddsImu input;
  auto bytes = m20::ros::serializeImu(input);
  bytes.pop_back();
  m20::ros::DrddsImu output;
  std::string error;
  EXPECT_FALSE(m20::ros::deserializeImu(bytes, output, error));
  EXPECT_FALSE(error.empty());
}
