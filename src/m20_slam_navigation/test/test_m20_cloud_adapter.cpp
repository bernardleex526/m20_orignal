#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstdint>
#include <cstring>
#include <string>

#include "m20_slam_navigation/ros/m20_cloud_adapter.hpp"

namespace
{

sensor_msgs::msg::PointField field(
  const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField value;
  value.name = name;
  value.offset = offset;
  value.datatype = datatype;
  value.count = 1;
  return value;
}

template<typename T>
void writeValue(std::uint8_t * point, std::size_t offset, const T & value)
{
  std::memcpy(point + offset, &value, sizeof(T));
}

sensor_msgs::msg::PointCloud2 makeM20Cloud(double header_time)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar_link";
  const auto stamp_ns = static_cast<std::int64_t>(header_time * 1e9);
  cloud.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  cloud.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  cloud.height = 1;
  cloud.width = 2;
  cloud.fields = {
    field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
    field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
    field("ring", 16, sensor_msgs::msg::PointField::UINT16),
    field("timestamp", 18, sensor_msgs::msg::PointField::FLOAT64),
  };
  cloud.point_step = 26;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0; i < 2; ++i) {
    auto * point = cloud.data.data() + i * cloud.point_step;
    writeValue(point, 0, static_cast<float>(i + 1));
    writeValue(point, 4, static_cast<float>(i + 2));
    writeValue(point, 8, static_cast<float>(i + 3));
    writeValue(point, 12, static_cast<float>(10 + i));
    writeValue(point, 16, static_cast<std::uint16_t>(20 + i));
    writeValue(point, 18, header_time + (i == 0 ? 0.01 : 0.09));
  }
  return cloud;
}

}  // namespace

TEST(M20CloudAdapter, ReadsOfficialRoboSenseContract)
{
  const auto result = m20::ros::adaptM20Cloud(makeM20Cloud(100.0));
  ASSERT_TRUE(result.error.empty()) << result.error;
  ASSERT_EQ(result.cloud->size(), 2U);
  ASSERT_EQ(result.point_time_offsets.size(), 2U);
  ASSERT_EQ(result.rings.size(), 2U);
  EXPECT_EQ(result.rings[0], 20U);
  EXPECT_EQ(result.rings[1], 21U);
  EXPECT_EQ(result.scan_start_ns, 100000000000LL);
  EXPECT_EQ(result.scan_end_ns, 100090000000LL);
  EXPECT_NEAR(result.point_time_offsets[0], 0.01, 1e-8);
  EXPECT_NEAR(result.point_time_offsets[1], 0.09, 1e-8);
  EXPECT_FLOAT_EQ(result.cloud->points[0].intensity, 10.0F);
}

TEST(M20CloudAdapter, UsesPointClockWhenHeaderIsInconsistent)
{
  auto cloud = makeM20Cloud(100.0);
  cloud.header.stamp.sec = 1;
  cloud.header.stamp.nanosec = 0;
  const auto result = m20::ros::adaptM20Cloud(cloud);
  ASSERT_TRUE(result.error.empty()) << result.error;
  EXPECT_TRUE(result.rewrote_header_stamp);
  EXPECT_EQ(result.scan_start_ns, 100010000000LL);
  EXPECT_NEAR(result.point_time_offsets[0], 0.0, 1e-8);
  EXPECT_NEAR(result.point_time_offsets[1], 0.08, 1e-8);
}

TEST(M20CloudAdapter, RejectsMissingTimestampField)
{
  auto cloud = makeM20Cloud(100.0);
  cloud.fields.pop_back();
  const auto result = m20::ros::adaptM20Cloud(cloud);
  EXPECT_FALSE(result.error.empty());
}

TEST(M20CloudAdapter, ClampsOfficialBagStyleRollback)
{
  const auto result = m20::ros::makeMonotonicStamp(1996800000LL, 2000000000LL);
  EXPECT_EQ(result.action, m20::ros::StampAction::CLAMP);
  EXPECT_EQ(result.rollback_ns, 3200000LL);
  EXPECT_EQ(result.stamp_ns, 2000000001LL);
}

TEST(M20CloudAdapter, DropsLargeClockReset)
{
  const auto result = m20::ros::makeMonotonicStamp(1000000000LL, 2000000000LL);
  EXPECT_EQ(result.action, m20::ros::StampAction::DROP);
}
