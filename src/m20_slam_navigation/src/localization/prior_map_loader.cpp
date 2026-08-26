#include "m20_slam_navigation/localization/prior_map_loader.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <limits>

namespace m20::localization {

PriorMapLoader::PriorMapLoader(const LocalizationParams& params)
    : params_(params) {}

bool PriorMapLoader::loadMap(const std::string& map_path) {
  full_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *full_map_) == -1) {
    loaded_ = false;
    return false;
  }

  // Downsample for efficient NDT matching
  downsampled_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(params_.map_voxel_leaf_size,
                 params_.map_voxel_leaf_size,
                 params_.map_voxel_leaf_size);
  vg.setInputCloud(full_map_);
  vg.filter(*downsampled_map_);

  loaded_ = true;
  return true;
}

PriorMapLoader::MapBounds PriorMapLoader::getBounds() const {
  MapBounds bounds;
  bounds.min = Eigen::Matrix<Scalar, 3, 1>::Constant(std::numeric_limits<Scalar>::max());
  bounds.max = Eigen::Matrix<Scalar, 3, 1>::Constant(-std::numeric_limits<Scalar>::max());

  if (!loaded_ || !full_map_) return bounds;

  for (const auto& pt : full_map_->points) {
    if (pt.x < bounds.min.x()) bounds.min.x() = pt.x;
    if (pt.y < bounds.min.y()) bounds.min.y() = pt.y;
    if (pt.z < bounds.min.z()) bounds.min.z() = pt.z;
    if (pt.x > bounds.max.x()) bounds.max.x() = pt.x;
    if (pt.y > bounds.max.y()) bounds.max.y() = pt.y;
    if (pt.z > bounds.max.z()) bounds.max.z() = pt.z;
  }
  return bounds;
}

}  // namespace m20::localization