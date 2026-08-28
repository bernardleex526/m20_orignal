#include "m20_slam_navigation/lio/voxel_map.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <unordered_set>

namespace m20::lio {

VoxelMap::VoxelMap(Scalar voxel_size, int max_voxels)
    : voxel_size_(voxel_size), max_voxels_(max_voxels) {
  level_resolution_.push_back(voxel_size_);
  voxels_by_level_.resize(1);
  voxels_by_level_.front().reserve(max_voxels_);
}

VoxelMap::VoxelMap(const LIOParams& params)
    : voxel_size_(params.voxel_size),
      max_voxels_(params.max_voxels),
      plane_threshold_(params.esti_plane_threshold),
      deepest_level_(std::max(0, params.deepest_level)),
      plane_level_(std::clamp(params.plane_level, 0, std::max(0, params.deepest_level))),
      top_level_(std::clamp(params.top_level, 0, std::max(0, params.deepest_level))) {
  if (voxel_size_ <= 0.0) voxel_size_ = 0.16;
  if (top_level_ > plane_level_) std::swap(top_level_, plane_level_);
  level_resolution_.resize(static_cast<std::size_t>(deepest_level_ + 1));
  voxels_by_level_.resize(static_cast<std::size_t>(deepest_level_ + 1));
  for (int level = 0; level <= deepest_level_; ++level) {
    level_resolution_[static_cast<std::size_t>(level)] =
      voxel_size_ * std::ldexp(Scalar(1.0), deepest_level_ - level);
    voxels_by_level_[static_cast<std::size_t>(level)].reserve(max_voxels_);
  }
}

VoxelKey VoxelMap::pointToKey(
    const Eigen::Matrix<Scalar, 3, 1>& p, int level) const {
  const Scalar inv = Scalar(1.0) / level_resolution_[static_cast<std::size_t>(level)];
  return {static_cast<int64_t>(std::floor(p.x() * inv)),
          static_cast<int64_t>(std::floor(p.y() * inv)),
          static_cast<int64_t>(std::floor(p.z() * inv))};
}

Eigen::Matrix<Scalar, 3, 1> VoxelMap::keyToCenter(
    const VoxelKey& key, int level) const {
  const Scalar resolution = level_resolution_[static_cast<std::size_t>(level)];
  return {(static_cast<Scalar>(key.ix) + Scalar(0.5)) * resolution,
          (static_cast<Scalar>(key.iy) + Scalar(0.5)) * resolution,
          (static_cast<Scalar>(key.iz) + Scalar(0.5)) * resolution};
}

void VoxelMap::insertCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const SE3Pose& T_world_lidar) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::unordered_set<VoxelKey, VoxelKeyHash>> touched(
    static_cast<std::size_t>(deepest_level_ + 1));
  for (const auto& pt : cloud->points) {
    const Eigen::Matrix<Scalar, 3, 1> body(pt.x, pt.y, pt.z);
    const Eigen::Matrix<Scalar, 3, 1> world = T_world_lidar.transformPoint(body);
    if (!world.allFinite()) continue;
    for (int level = top_level_; level <= deepest_level_; ++level) {
      const auto key = pointToKey(world, level);
      voxels_by_level_[static_cast<std::size_t>(level)][key].addPoint(world);
      touched[static_cast<std::size_t>(level)].insert(key);
    }
  }
  for (int level = top_level_; level <= deepest_level_; ++level) {
    auto& voxels = voxels_by_level_[static_cast<std::size_t>(level)];
    for (const auto& key : touched[static_cast<std::size_t>(level)]) {
      voxels.at(key).updatePlane();
    }
  }
  if (static_cast<int>(voxels_by_level_[static_cast<std::size_t>(deepest_level_)].size()) >
      max_voxels_) {
    pruneUnlocked();
  }
}

bool VoxelMap::findPlane(
    const Eigen::Matrix<Scalar, 3, 1>& point, PlaneMatch& match) const {
  std::vector<PlaneMatch> matches;
  std::vector<std::uint8_t> valid;
  findPlanes({point}, matches, valid);
  if (valid.empty() || valid.front() == 0U) return false;
  match = matches.front();
  return true;
}

void VoxelMap::findPlanes(
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& points,
    std::vector<PlaneMatch>& matches,
    std::vector<std::uint8_t>& valid) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  matches.assign(points.size(), PlaneMatch{});
  valid.assign(points.size(), 0U);
  static constexpr int kVendorStencil[7][3] = {
    {0, 0, 0},
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1},
  };

  for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
    const auto& point = points[point_index];
    Scalar best_abs_residual = plane_threshold_;
    for (int level = plane_level_; level >= top_level_; --level) {
      const auto& voxels = voxels_by_level_[static_cast<std::size_t>(level)];
      const VoxelKey center = pointToKey(point, level);
      for (const auto & offset : kVendorStencil) {
        const VoxelKey key{
          center.ix + offset[0], center.iy + offset[1], center.iz + offset[2]};
        const auto voxel = voxels.find(key);
        if (voxel == voxels.end() || !voxel->second.plane_valid) {
          continue;
        }
        const Scalar residual =
          voxel->second.plane_normal.dot(point) + voxel->second.plane_offset;
        const Scalar abs_residual = std::abs(residual);
        if (abs_residual < best_abs_residual) {
          best_abs_residual = abs_residual;
          auto& output = matches[point_index];
          output.normal = voxel->second.plane_normal;
          output.offset = voxel->second.plane_offset;
          output.residual = residual;
          output.level = level;
          output.support_points = voxel->second.point_count;
          valid[point_index] = 1U;
        }
      }
    }
  }
}

std::vector<std::pair<int, const VoxelEntry*>> VoxelMap::findCorrespondences(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source, Scalar radius) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::pair<int, const VoxelEntry*>> result;
  result.reserve(source->size());
  const int level = deepest_level_;
  const auto& voxels = voxels_by_level_[static_cast<std::size_t>(level)];
  const int span = static_cast<int>(std::ceil(
    radius / level_resolution_[static_cast<std::size_t>(level)]));
  const Scalar radius_sq = radius * radius;
  for (int index = 0; index < static_cast<int>(source->size()); ++index) {
    const auto& pt = source->points[static_cast<std::size_t>(index)];
    const Eigen::Matrix<Scalar, 3, 1> point(pt.x, pt.y, pt.z);
    const VoxelKey center = pointToKey(point, level);
    const VoxelEntry* best = nullptr;
    Scalar best_distance = radius_sq;
    for (int dx = -span; dx <= span; ++dx) {
      for (int dy = -span; dy <= span; ++dy) {
        for (int dz = -span; dz <= span; ++dz) {
          const auto it = voxels.find({center.ix + dx, center.iy + dy, center.iz + dz});
          if (it == voxels.end()) continue;
          const Scalar distance = (point - it->second.centroid).squaredNorm();
          if (distance < best_distance) {
            best_distance = distance;
            best = &it->second;
          }
        }
      }
    }
    if (best) result.emplace_back(index, best);
  }
  return result;
}

const VoxelEntry* VoxelMap::getVoxel(
    const Eigen::Matrix<Scalar, 3, 1>& point) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  const int level = deepest_level_;
  const auto& voxels = voxels_by_level_[static_cast<std::size_t>(level)];
  const auto it = voxels.find(pointToKey(point, level));
  return it == voxels.end() ? nullptr : &it->second;
}

std::vector<std::pair<VoxelKey, VoxelEntry>> VoxelMap::getAllVoxels() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  const auto& voxels = voxels_by_level_[static_cast<std::size_t>(deepest_level_)];
  std::vector<std::pair<VoxelKey, VoxelEntry>> result;
  result.reserve(voxels.size());
  for (const auto& item : voxels) result.push_back(item);
  return result;
}

void VoxelMap::prune() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  pruneUnlocked();
}

void VoxelMap::pruneUnlocked() {
  for (int level = top_level_; level <= deepest_level_; ++level) {
    auto& voxels = voxels_by_level_[static_cast<std::size_t>(level)];
    if (static_cast<int>(voxels.size()) <= max_voxels_) continue;
    std::vector<std::pair<Scalar, VoxelKey>> distances;
    distances.reserve(voxels.size());
    for (const auto& item : voxels) {
      distances.emplace_back(keyToCenter(item.first, level).squaredNorm(), item.first);
    }
    std::nth_element(distances.begin(), distances.begin() + max_voxels_, distances.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    for (auto it = distances.begin() + max_voxels_; it != distances.end(); ++it) {
      voxels.erase(it->second);
    }
  }
}

void VoxelMap::clear() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  for (auto& level : voxels_by_level_) level.clear();
}

std::size_t VoxelMap::size() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return voxels_by_level_[static_cast<std::size_t>(deepest_level_)].size();
}

}  // namespace m20::lio
