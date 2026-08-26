#pragma once
/**
 * @file elevation_grid.hpp
 * @brief 2.5D elevation grid map for terrain traversability analysis.
 *
 * Maintains a local multi-layer grid map centered on the robot.
 * Each cell stores height statistics (min, max, mean, variance).
 * Grid is implemented as a rolling buffer that shifts with the robot.
 *
 * Grid dimensions: W×H cells of resolution r.
 * Coverage: W·r × H·r meters, typically 100m × 100m at 0.05m res.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace m20::terrain {

class ElevationGrid {
public:
  ElevationGrid(const TerrainParams& params, Scalar grid_length = 100.0);

  /**
   * @brief Update grid with new point cloud.
   *
   * Points are first transformed to world/odom frame, then projected onto
   * the grid. Each cell's elevation statistics are updated incrementally.
   *
   * @param cloud       Deskewed LiDAR point cloud
   * @param T_world_lidar  LiDAR-to-world transform
   */
  void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
              const SE3Pose& T_world_lidar);

  /**
   * @brief Shift grid to keep robot at center.
   */
  void shiftToOrigin(const Eigen::Matrix<Scalar, 2, 1>& robot_xy);

  /// Access cell at world coordinates
  const ElevationCell* cellAt(Scalar x, Scalar y) const;
  ElevationCell* mutableCellAt(Scalar x, Scalar y);

  /// Grid properties
  Scalar resolution() const { return resolution_; }
  int    width()      const { return width_; }
  int    height()     const { return height_; }
  const Eigen::Matrix<Scalar, 2, 1>& origin() const { return origin_; }

  /// Get all cells (for costmap export)
  const std::vector<ElevationCell>& cells() const { return cells_; }

  /// Convert world coordinates to grid indices
  bool worldToGrid(Scalar x, Scalar y, int& gx, int& gy) const;

  /// Convert grid indices to world coordinates (cell center)
  Eigen::Matrix<Scalar, 2, 1> gridToWorld(int gx, int gy) const;

private:
  TerrainParams              params_;
  Scalar                     resolution_;
  int                        width_;
  int                        height_;
  Eigen::Matrix<Scalar, 2, 1> origin_{0.0, 0.0};  ///< grid origin (world coords of lower-left corner)
  std::vector<ElevationCell> cells_;
};

}  // namespace m20::terrain