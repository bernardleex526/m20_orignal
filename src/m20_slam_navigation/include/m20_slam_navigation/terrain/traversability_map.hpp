#pragma once
/**
 * @file traversability_map.hpp
 * @brief Orchestrator for terrain traversability analysis pipeline.
 *
 * Pipeline:
 *   1. ElevationGrid:  update with deskewed cloud → 2.5D elevation stats.
 *   2. SlopeAnalyzer:  PCA normal estimation → slope angle per cell.
 *   3. RoughnessAnalyzer: σ_z computation → roughness per cell.
 *   4. StepDetector:   neighbour height diff → step obstacles flagged.
 *   5. RayOcclusion (inline): line-of-sight check for overhangs/drops.
 *
 * Final cost per cell:
 *   cost = w_slope·c_slope + w_roughness·c_roughness + w_step·c_step
 *        + w_occlusion·c_occlusion
 *
 * Output is a 2D costmap (grid of uint8_t costs) suitable for Nav2 costmap layers.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/terrain/elevation_grid.hpp"
#include "m20_slam_navigation/terrain/slope_analyzer.hpp"
#include "m20_slam_navigation/terrain/roughness_analyzer.hpp"
#include "m20_slam_navigation/terrain/step_detector.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <mutex>
#include <vector>

namespace m20::terrain {

class TraversabilityMap {
public:
  TraversabilityMap(const TerrainParams& terrain_params);

  /**
   * @brief Full pipeline: update elevation grid and compute traversability.
   *
   * @param cloud           Deskewed LiDAR point cloud
   * @param T_world_lidar   LiDAR-to-world transform
   */
  void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
              const SE3Pose& T_world_lidar);

  /**
   * @brief Export traversability cost as 2D occupancy grid (uint8_t).
   *
   * @return Flat vector of costs [0, 255] in row-major order.
   *         255 = lethal (untraversable), 0 = free (perfectly traversable).
   */
  std::vector<uint8_t> exportCostmap() const;

  /// Get reference to elevation grid
  const ElevationGrid& getElevationGrid() const { return *elevation_grid_; }

  /// Get grid dimensions for costmap export
  int width()  const;
  int height() const;
  Scalar resolution() const;

  /// Query traversability cost at world coordinate
  uint8_t costAt(Scalar x, Scalar y) const;

  /// Get centroid position for shifting
  void shiftTo(const Eigen::Matrix<Scalar, 2, 1>& robot_xy);

private:
  TerrainParams                         params_;
  std::unique_ptr<ElevationGrid>        elevation_grid_;
  std::unique_ptr<SlopeAnalyzer>        slope_analyzer_;
  std::unique_ptr<RoughnessAnalyzer>    roughness_analyzer_;
  std::unique_ptr<StepDetector>         step_detector_;
  mutable std::mutex                    mutex_;
};

}  // namespace m20::terrain