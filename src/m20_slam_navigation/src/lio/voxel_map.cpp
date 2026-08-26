#include "m20_slam_navigation/lio/voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace m20::lio {

VoxelMap::VoxelMap(Scalar voxel_size, int max_voxels)
    : voxel_size_(voxel_size), voxel_inv_(1.0 / voxel_size), max_voxels_(max_voxels) {
  voxels_.reserve(max_voxels_);
}

VoxelKey VoxelMap::pointToKey(const Eigen::Matrix<Scalar, 3, 1>& p) const {
  return {
    static_cast<int64_t>(std::floor(p.x() * voxel_inv_)),
    static_cast<int64_t>(std::floor(p.y() * voxel_inv_)),
    static_cast<int64_t>(std::floor(p.z() * voxel_inv_))
  };
}

Eigen::Matrix<Scalar, 3, 1> VoxelMap::keyToCenter(const VoxelKey& key) const {
  return {
    (static_cast<Scalar>(key.ix) + Scalar(0.5)) * voxel_size_,
    (static_cast<Scalar>(key.iy) + Scalar(0.5)) * voxel_size_,
    (static_cast<Scalar>(key.iz) + Scalar(0.5)) * voxel_size_
  };
}

void VoxelMap::insertCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const SE3Pose& T_world_lidar) {

  for (const auto& pt : cloud->points) {
    // Transform point to world frame
    Eigen::Matrix<Scalar, 3, 1> p_lidar(pt.x, pt.y, pt.z);
    Eigen::Matrix<Scalar, 3, 1> p_world = T_world_lidar.transformPoint(p_lidar);

    VoxelKey key = pointToKey(p_world);
    voxels_[key].addPoint(p_world);
  }

  // Prune if exceeding max count
  if (static_cast<int>(voxels_.size()) > max_voxels_) {
    prune();
  }
}

std::vector<std::pair<int, const VoxelEntry*>> VoxelMap::findCorrespondences(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    Scalar radius) const {

  std::vector<std::pair<int, const VoxelEntry*>> correspondences;
  correspondences.reserve(source->size());

  Scalar radius_sq = radius * radius;
  int voxel_span = static_cast<int>(std::ceil(radius * voxel_inv_));

  for (int idx = 0; idx < static_cast<int>(source->size()); ++idx) {
    const auto& pt = source->points[idx];
    Eigen::Matrix<Scalar, 3, 1> p(pt.x, pt.y, pt.z);

    // Search neighbouring voxels
    VoxelKey center_key = pointToKey(p);
    const VoxelEntry* best_voxel = nullptr;
    Scalar best_dist_sq = radius_sq;

    for (int dx = -voxel_span; dx <= voxel_span; ++dx) {
      for (int dy = -voxel_span; dy <= voxel_span; ++dy) {
        for (int dz = -voxel_span; dz <= voxel_span; ++dz) {
          VoxelKey nkey{center_key.ix + dx, center_key.iy + dy, center_key.iz + dz};
          auto it = voxels_.find(nkey);
          if (it == voxels_.end()) continue;

          Scalar dist_sq = (p - it->second.centroid).squaredNorm();
          if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_voxel = &it->second;
          }
        }
      }
    }

    if (best_voxel) {
      correspondences.push_back({idx, best_voxel});
    }
  }

  return correspondences;
}

const VoxelEntry* VoxelMap::getVoxel(const Eigen::Matrix<Scalar, 3, 1>& point) const {
  VoxelKey key = pointToKey(point);
  auto it = voxels_.find(key);
  return (it != voxels_.end()) ? &it->second : nullptr;
}

std::vector<std::pair<VoxelKey, VoxelEntry>> VoxelMap::getAllVoxels() const {
  std::vector<std::pair<VoxelKey, VoxelEntry>> result;
  result.reserve(voxels_.size());
  for (const auto& [key, entry] : voxels_) {
    result.push_back({key, entry});
  }
  return result;
}

void VoxelMap::prune() {
  // Remove voxels farthest from origin (simple pruning strategy)
  if (static_cast<int>(voxels_.size()) <= max_voxels_) return;

  // Collect all voxels with distance from origin
  std::vector<std::pair<Scalar, VoxelKey>> distances;
  distances.reserve(voxels_.size());
  for (const auto& [key, entry] : voxels_) {
    Scalar dist = keyToCenter(key).norm();
    distances.push_back({dist, key});
  }

  // Sort by distance descending
  std::sort(distances.begin(), distances.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

  // Keep only the closest max_voxels_
  std::size_t to_remove = voxels_.size() - max_voxels_;
  for (std::size_t i = 0; i < to_remove; ++i) {
    voxels_.erase(distances[i].second);
  }
}

void VoxelMap::clear() {
  voxels_.clear();
}

}  // namespace m20::lio