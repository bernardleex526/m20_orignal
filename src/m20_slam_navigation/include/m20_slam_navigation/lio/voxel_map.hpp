#pragma once
/**
 * @file voxel_map.hpp
 * @brief Hash-based incremental 3D voxel grid for VGICP.
 *
 * Uses spatial hashing for O(1) voxel access. Each voxel stores a Gaussian
 * distribution (centroid μ, covariance Σ) updated incrementally via Welford's
 * online algorithm.
 *
 * The hash function maps 3D integer voxel coordinates to a flat bucket index:
 *   hash(ix, iy, iz) = (ix * P1) ^ (iy * P2) ^ (iz * P3)  mod N
 * with large primes P1, P2, P3.
 *
 * Only occupied voxels are stored (sparse map).
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Dense>

#include <memory>
#include <unordered_map>
#include <vector>

namespace m20::lio {

/// 3D integer voxel coordinate key
struct VoxelKey {
  int64_t ix, iy, iz;

  bool operator==(const VoxelKey& o) const {
    return ix == o.ix && iy == o.iy && iz == o.iz;
  }
};

/// Hash for VoxelKey
struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const {
    // Large primes for spatial hashing
    constexpr std::size_t P1 = 73856093;
    constexpr std::size_t P2 = 19349663;
    constexpr std::size_t P3 = 83492791;
    return (static_cast<std::size_t>(k.ix) * P1) ^
           (static_cast<std::size_t>(k.iy) * P2) ^
           (static_cast<std::size_t>(k.iz) * P3);
  }
};

class VoxelMap {
public:
  /// @param voxel_size  Edge length of each voxel [m]
  /// @param max_voxels  Max number of voxels before pruning
  explicit VoxelMap(Scalar voxel_size = 0.5, int max_voxels = 100000);

  /**
   * @brief Insert a point cloud into the voxel map.
   *
   * For each point, finds or creates its voxel and updates the Gaussian.
   * Points are transformed to world frame before insertion.
   *
   * @param cloud      Input point cloud
   * @param T_world_lidar  LiDAR-to-world transform
   */
  void insertCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                   const SE3Pose& T_world_lidar);

  /**
   * @brief Find nearest voxel distributions for each source point.
   *
   * For VGICP registration: each source point queries neighbouring voxels
   * within a radius to form distribution-to-distribution correspondences.
   *
   * @param source        Source point cloud (in world frame)
   * @param radius        Search radius [m]
   * @return              List of (source_point_index, voxel_ptr) pairs
   */
  std::vector<std::pair<int, const VoxelEntry*>> findCorrespondences(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
      Scalar radius) const;

  /// Get voxel at a 3D point (returns nullptr if not occupied)
  const VoxelEntry* getVoxel(const Eigen::Matrix<Scalar, 3, 1>& point) const;

  /// Return all occupied voxels (for loop closure / submap extraction)
  std::vector<std::pair<VoxelKey, VoxelEntry>> getAllVoxels() const;

  /// Prune voxels farthest from origin when exceeding max count
  void prune();

  /// Clear all voxels
  void clear();

  /// Number of occupied voxels
  std::size_t size() const { return voxels_.size(); }

  /// Voxel resolution
  Scalar resolution() const { return voxel_size_; }

private:
  VoxelKey pointToKey(const Eigen::Matrix<Scalar, 3, 1>& p) const;
  Eigen::Matrix<Scalar, 3, 1> keyToCenter(const VoxelKey& key) const;

  Scalar voxel_size_;
  Scalar voxel_inv_;
  int    max_voxels_;

  std::unordered_map<VoxelKey, VoxelEntry, VoxelKeyHash> voxels_;
};

}  // namespace m20::lio