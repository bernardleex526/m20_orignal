#include "m20_slam_navigation/lio/vendor_output_contract.hpp"

#include <pcl/filters/voxel_grid.h>

#include <cmath>

namespace m20::lio {

VendorCloudProducts makeVendorCloudProducts(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & deskewed,
  const std::vector<double> & point_time_offsets,
  const std::vector<std::uint16_t> & rings,
  Scalar min_range,
  Scalar max_range,
  bool enable_downsample,
  Scalar registration_leaf_size,
  Scalar body_leaf_size)
{
  VendorCloudProducts products;
  products.registration_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
  products.body_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
  products.registration_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr annotated(
    new pcl::PointCloud<pcl::PointXYZINormal>());
  annotated->reserve(deskewed ? deskewed->size() : 0U);
  const Scalar min_range_sq = min_range * min_range;
  const Scalar max_range_sq = max_range * max_range;
  if (deskewed) {
    for (std::size_t index = 0; index < deskewed->size(); ++index) {
      const auto & point = deskewed->points[index];
      const Scalar range_sq = point.x * point.x + point.y * point.y + point.z * point.z;
      if (!std::isfinite(range_sq) || range_sq < min_range_sq || range_sq > max_range_sq) {
        continue;
      }
      pcl::PointXYZINormal vendor_point{};
      vendor_point.x = point.x;
      vendor_point.y = point.y;
      vendor_point.z = point.z;
      vendor_point.normal_x = 0.0F;
      vendor_point.normal_y = 0.0F;
      vendor_point.normal_z = static_cast<float>(index < rings.size() ? rings[index] : 0U);
      vendor_point.intensity = point.intensity;
      vendor_point.curvature = static_cast<float>(
        (index < point_time_offsets.size() ? point_time_offsets[index] : 0.0) * 1000.0);
      annotated->push_back(vendor_point);
    }
  }
  annotated->width = static_cast<std::uint32_t>(annotated->size());
  annotated->height = 1;
  annotated->is_dense = false;

  if (body_leaf_size > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZINormal> body_filter;
    const float leaf = static_cast<float>(body_leaf_size);
    body_filter.setLeafSize(leaf, leaf, leaf);
    body_filter.setDownsampleAllData(true);
    body_filter.setInputCloud(annotated);
    body_filter.filter(*products.body_cloud);
  } else {
    *products.body_cloud = *annotated;
  }
  products.body_cloud->is_dense = false;

  if (enable_downsample && registration_leaf_size > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZINormal> registration_filter;
    const float leaf = static_cast<float>(registration_leaf_size);
    registration_filter.setLeafSize(leaf, leaf, leaf);
    registration_filter.setDownsampleAllData(true);
    registration_filter.setInputCloud(annotated);
    registration_filter.filter(*products.registration_cloud);
  } else {
    *products.registration_cloud = *annotated;
  }

  products.registration_points->reserve(products.registration_cloud->size());
  for (const auto & point : products.registration_cloud->points) {
    pcl::PointXYZI registration_point;
    registration_point.x = point.x;
    registration_point.y = point.y;
    registration_point.z = point.z;
    registration_point.intensity = point.intensity;
    products.registration_points->push_back(registration_point);
  }
  products.registration_points->width =
    static_cast<std::uint32_t>(products.registration_points->size());
  products.registration_points->height = 1;
  products.registration_points->is_dense = true;
  return products;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr makeVendorAlignedCloud(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & input,
  const SE3Pose & pose,
  const std::shared_ptr<VoxelMap> & voxel_map)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(
    new pcl::PointCloud<pcl::PointXYZINormal>());
  output->reserve(input ? input->size() : 0U);
  std::vector<Eigen::Matrix<Scalar, 3, 1>> points_world;
  points_world.reserve(input ? input->size() : 0U);
  if (input) {
    for (const auto & point : input->points) {
      points_world.push_back(pose.transformPoint(
        Eigen::Matrix<Scalar, 3, 1>(point.x, point.y, point.z)));
    }
  }

  std::vector<VoxelMap::PlaneMatch> matches;
  std::vector<std::uint8_t> valid;
  if (voxel_map && voxel_map->size() > 0U) {
    voxel_map->findPlanes(points_world, matches, valid);
  } else {
    matches.resize(points_world.size());
    valid.assign(points_world.size(), 0U);
  }
  for (std::size_t index = 0; index < points_world.size(); ++index) {
    const auto & point = input->points[index];
    const auto & transformed = points_world[index];
    pcl::PointXYZINormal result{};
    result.x = static_cast<float>(transformed.x());
    result.y = static_cast<float>(transformed.y());
    result.z = static_cast<float>(transformed.z());
    if (index < valid.size() && valid[index] != 0U) {
      result.normal_x = static_cast<float>(matches[index].normal.x());
      result.normal_y = static_cast<float>(matches[index].normal.y());
      result.normal_z = static_cast<float>(matches[index].normal.z());
      result.intensity = static_cast<float>(matches[index].offset);
    } else {
      result.normal_x = 0.0F;
      result.normal_y = 0.0F;
      result.normal_z = 0.0F;
      result.intensity = point.intensity;
    }
    result.curvature = point.normal_z;
    output->push_back(result);
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;
  return output;
}

}  // namespace m20::lio
