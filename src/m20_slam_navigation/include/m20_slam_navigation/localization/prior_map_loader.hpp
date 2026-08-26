#pragma once
/**
 * @file prior_map_loader.hpp
 * @brief Loads and manages a pre-built static high-density point cloud map.
 *
 * Supports .pcd format. The map is downsampled with a voxel grid filter
 * for efficient NDT matching. The original high-density map is kept for
 * high-accuracy relocalization.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>

namespace m20::localization {

class PriorMapLoader {
public:
  PriorMapLoader(const LocalizationParams& params);

  /**
   * @brief Load point cloud map from file.
   * @param map_path  Path to .pcd file
   * @return true on success
   */
  bool loadMap(const std::string& map_path);

  /// Get the full-resolution map
  pcl::PointCloud<pcl::PointXYZ>::Ptr getFullMap() const { return full_map_; }

  /// Get the downsampled map for NDT
  pcl::PointCloud<pcl::PointXYZ>::Ptr getDownsampledMap() const { return downsampled_map_; }

  /// Check if map is loaded
  bool isLoaded() const { return loaded_; }

  /// Get map bounds
  struct MapBounds {
    Eigen::Matrix<Scalar, 3, 1> min;
    Eigen::Matrix<Scalar, 3, 1> max;
  };
  MapBounds getBounds() const;

private:
  LocalizationParams                     params_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr    full_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr    downsampled_map_;
  bool                                   loaded_{false};
};

}  // namespace m20::localization