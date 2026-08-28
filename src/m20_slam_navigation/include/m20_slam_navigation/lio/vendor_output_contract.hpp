#pragma once

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/lio/voxel_map.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace m20::lio {

struct VendorCloudProducts {
  pcl::PointCloud<pcl::PointXYZI>::Ptr registration_points;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr body_cloud;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr registration_cloud;
};

/// Build the two native PointXYZINormal output clouds and the PointXYZI cloud
/// consumed by the ESKF update from one end-frame deskewed scan.
VendorCloudProducts makeVendorCloudProducts(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & deskewed,
  const std::vector<double> & point_time_offsets,
  const std::vector<std::uint16_t> & rings,
  Scalar min_range,
  Scalar max_range,
  bool enable_downsample,
  Scalar registration_leaf_size,
  Scalar body_leaf_size);

/// Transform the registration cloud to map and replace the auxiliary fields
/// with the native matched-plane contract.
pcl::PointCloud<pcl::PointXYZINormal>::Ptr makeVendorAlignedCloud(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & input,
  const SE3Pose & pose,
  const std::shared_ptr<VoxelMap> & voxel_map);

}  // namespace m20::lio
