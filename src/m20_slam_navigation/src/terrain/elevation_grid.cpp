#include "m20_slam_navigation/terrain/elevation_grid.hpp"

#include <algorithm>
#include <cmath>

namespace m20::terrain {

ElevationGrid::ElevationGrid(const TerrainParams& params, Scalar grid_length)
    : params_(params)
    , resolution_(params.grid_resolution) {

  int size = static_cast<int>(std::ceil(grid_length / resolution_));
  width_  = size;
  height_ = size;

  cells_.resize(width_ * height_);

  // Center grid on origin
  Scalar half = grid_length * 0.5;
  origin_ = Eigen::Matrix<Scalar, 2, 1>(-half, -half);
}

void ElevationGrid::update(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const SE3Pose& T_world_lidar) {

  for (const auto& pt : cloud->points) {
    // Transform to world frame
    Eigen::Matrix<Scalar, 3, 1> p_lidar(pt.x, pt.y, pt.z);
    Eigen::Matrix<Scalar, 3, 1> p_world = T_world_lidar.transformPoint(p_lidar);

    // Range filter
    Scalar range = std::sqrt(p_world.x() * p_world.x() + p_world.y() * p_world.y());
    if (range < params_.min_range || range > params_.max_range) continue;

    // Project to grid
    int gx, gy;
    if (!worldToGrid(p_world.x(), p_world.y(), gx, gy)) continue;

    ElevationCell& cell = cells_[gy * width_ + gx];
    cell.update(p_world.z());
  }
}

void ElevationGrid::shiftToOrigin(const Eigen::Matrix<Scalar, 2, 1>& robot_xy) {
  // Compute shift in grid cells
  Scalar dx = robot_xy.x() - origin_.x() - 0.5 * width_ * resolution_;
  Scalar dy = robot_xy.y() - origin_.y() - 0.5 * height_ * resolution_;

  int shift_x = static_cast<int>(std::floor(dx / resolution_));
  int shift_y = static_cast<int>(std::floor(dy / resolution_));

  if (shift_x == 0 && shift_y == 0) return;

  // Create new grid with updated origin
  std::vector<ElevationCell> new_cells(width_ * height_);
  Eigen::Matrix<Scalar, 2, 1> new_origin(
      origin_.x() + shift_x * resolution_,
      origin_.y() + shift_y * resolution_);

  // Copy overlapping cells
  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      int old_gx = gx + shift_x;
      int old_gy = gy + shift_y;
      if (old_gx >= 0 && old_gx < width_ && old_gy >= 0 && old_gy < height_) {
        new_cells[gy * width_ + gx] = cells_[old_gy * width_ + old_gx];
      }
    }
  }

  cells_ = std::move(new_cells);
  origin_ = new_origin;
}

const ElevationCell* ElevationGrid::cellAt(Scalar x, Scalar y) const {
  int gx, gy;
  if (!worldToGrid(x, y, gx, gy)) return nullptr;
  return &cells_[gy * width_ + gx];
}

ElevationCell* ElevationGrid::mutableCellAt(Scalar x, Scalar y) {
  int gx, gy;
  if (!worldToGrid(x, y, gx, gy)) return nullptr;
  return &cells_[gy * width_ + gx];
}

bool ElevationGrid::worldToGrid(Scalar x, Scalar y, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((x - origin_.x()) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_.y()) / resolution_));
  return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

Eigen::Matrix<Scalar, 2, 1> ElevationGrid::gridToWorld(int gx, int gy) const {
  return {
    origin_.x() + (static_cast<Scalar>(gx) + 0.5) * resolution_,
    origin_.y() + (static_cast<Scalar>(gy) + 0.5) * resolution_
  };
}

}  // namespace m20::terrain