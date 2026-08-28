#include "m20_slam_navigation/lio/vendor_output_contract.hpp"
#include "m20_slam_navigation/ros/vendor_output_contract.hpp"

#include <gtest/gtest.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <string>

namespace {

TEST(VendorOutputContract, BodyCloudPreservesRingAndRelativeMilliseconds)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr deskewed(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointXYZI first;
  first.x = 1.0F;
  first.y = 0.0F;
  first.z = 0.0F;
  first.intensity = 11.0F;
  deskewed->push_back(first);
  pcl::PointXYZI second;
  second.x = 2.0F;
  second.y = 0.0F;
  second.z = 0.0F;
  second.intensity = 22.0F;
  deskewed->push_back(second);

  const auto products = m20::lio::makeVendorCloudProducts(
    deskewed, {0.001, 0.099961}, {1U, 191U},
    0.2, 30.0, false, 0.15, 0.0);

  ASSERT_EQ(products.body_cloud->size(), 2U);
  EXPECT_FLOAT_EQ(products.body_cloud->points[0].normal_x, 0.0F);
  EXPECT_FLOAT_EQ(products.body_cloud->points[0].normal_y, 0.0F);
  EXPECT_FLOAT_EQ(products.body_cloud->points[0].normal_z, 1.0F);
  EXPECT_FLOAT_EQ(products.body_cloud->points[0].intensity, 11.0F);
  EXPECT_NEAR(products.body_cloud->points[0].curvature, 1.0F, 1e-5F);
  EXPECT_FLOAT_EQ(products.body_cloud->points[1].normal_z, 191.0F);
  EXPECT_FLOAT_EQ(products.body_cloud->points[1].intensity, 22.0F);
  EXPECT_NEAR(products.body_cloud->points[1].curvature, 99.961F, 1e-4F);
  EXPECT_FALSE(products.body_cloud->is_dense);

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*products.body_cloud, message);
  EXPECT_EQ(message.point_step, 48U);
  const auto has_field = [&message](const std::string & name) {
      return std::any_of(message.fields.begin(), message.fields.end(),
        [&name](const auto & field) {return field.name == name;});
    };
  EXPECT_TRUE(has_field("normal_x"));
  EXPECT_TRUE(has_field("normal_y"));
  EXPECT_TRUE(has_field("normal_z"));
  EXPECT_TRUE(has_field("intensity"));
  EXPECT_TRUE(has_field("curvature"));
}

TEST(VendorOutputContract, AlignedCloudCarriesPlaneAndDownsampledRing)
{
  m20::LIOParams params;
  params.voxel_size = 0.16;
  params.deepest_level = 2;
  params.plane_level = 2;
  params.top_level = 1;
  params.esti_plane_threshold = 0.1;
  auto map = std::make_shared<m20::lio::VoxelMap>(params);
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane(new pcl::PointCloud<pcl::PointXYZI>());
  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(x) * 0.02F;
      point.y = static_cast<float>(y) * 0.02F;
      point.z = 0.0F;
      plane->push_back(point);
    }
  }
  map->insertCloud(plane, m20::SE3Pose::Identity());

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr registration(
    new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointXYZINormal point{};
  point.x = 0.01F;
  point.y = 0.01F;
  point.z = 0.02F;
  point.normal_z = 37.0F;
  registration->push_back(point);

  const auto aligned = m20::lio::makeVendorAlignedCloud(
    registration, m20::SE3Pose::Identity(), map);
  ASSERT_EQ(aligned->size(), 1U);
  EXPECT_NEAR(std::abs(aligned->points[0].normal_z), 1.0F, 1e-4F);
  EXPECT_NEAR(aligned->points[0].intensity, 0.0F, 1e-4F);
  EXPECT_FLOAT_EQ(aligned->points[0].curvature, 37.0F);
  EXPECT_TRUE(aligned->is_dense);
}

TEST(VendorOutputContract, DisabledAuxiliaryOutputsMatchNativeRuntime)
{
  const auto policy = m20::ros::vendorAuxiliaryOutputPolicy(false, false);
  EXPECT_FALSE(policy.publish_depth_cloud);
  EXPECT_TRUE(policy.publish_accumulated_cloud);
  EXPECT_FALSE(policy.populate_accumulated_cloud);

  pcl::PointCloud<pcl::PointXYZ> empty;
  empty.width = 0;
  empty.height = 1;
  empty.is_dense = true;
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(empty, message);
  EXPECT_EQ(message.point_step, 16U);
  EXPECT_EQ(message.width, 0U);
  EXPECT_EQ(message.height, 1U);
  EXPECT_TRUE(message.is_dense);
  ASSERT_EQ(message.fields.size(), 3U);
  EXPECT_EQ(message.fields[0].name, "x");
  EXPECT_EQ(message.fields[1].name, "y");
  EXPECT_EQ(message.fields[2].name, "z");
  EXPECT_EQ(message.header.stamp.sec, 0);
  EXPECT_EQ(message.header.stamp.nanosec, 0U);
}

}  // namespace
