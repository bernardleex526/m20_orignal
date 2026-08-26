#include "m20_slam_navigation/terrain/traversability_map.hpp"

#include <algorithm>
#include <cmath>

namespace m20::terrain {

TraversabilityMap::TraversabilityMap(const TerrainParams& terrain_params)
    : params_(terrain_params)
    , elevation_grid_(std::make_unique<ElevationGrid>(terrain_params))
    , slope_analyzer_(std::make_unique<SlopeAnalyzer>(terrain_params))
    , roughness_analyzer_(std::make_unique<RoughnessAnalyzer>(terrain_params))
    , step_detector_(std::make_unique<StepDetector>(terrain_params)) {
}

void TraversabilityMap::update(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const SE3Pose& T_world_lidar) {

  std::lock_guard<std::mutex> lock(mutex_);

  // Step 1: Update elevation grid
  elevation_grid_->update(cloud, T_world_lidar);

  // Step 2: Shift grid to keep robot at center
  Eigen::Matrix<Scalar, 2, 1> robot_xy(T_world_lidar.t.x(), T_world_lidar.t.y());
  elevation_grid_->shiftToOrigin(robot_xy);

  // Step 3-5: Analyze terrain
  slope_analyzer_->analyze(*elevation_grid_, cloud);
  roughness_analyzer_->analyze(*elevation_grid_);
  step_detector_->analyze(*elevation_grid_);

  // Step 6: Occlusion check (simplified ray visibility)
  // For each cell, check if there's a clear line of sight from robot height
  // TODO: full occlusion check
}

std::vector<uint8_t> TraversabilityMap::exportCostmap() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<uint8_t> costmap(elevation_grid_->cells().size(), 0);

  for (std::size_t i = 0; i < elevation_grid_->cells().size(); ++i) {
    const auto& cell = elevation_grid_->cells()[i];

    if (cell.n_points == 0) {
      costmap[i] = 255;  // unknown → treat as lethal
    } else if (!cell.traversable) {
      costmap[i] = 254;  // lethal
    } else {
      // Clamp cost to [0, 253]
      Scalar cost = cell.cost;
      cost = std::min(cost, Scalar(253));
      costmap[i] = static_cast<uint8_t>(cost);
    }
  }

  return costmap;
}

int TraversabilityMap::width() const {
  return elevation_grid_->width();
}

int TraversabilityMap::height() const {
  return elevation_grid_->height();
}

Scalar TraversabilityMap::resolution() const {
  return elevation_grid_->resolution();
}

uint8_t TraversabilityMap::costAt(Scalar x, Scalar y) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const ElevationCell* cell = elevation_grid_->cellAt(x, y);
  if (!cell || cell->n_points == 0) return 255;
  if (!cell->traversable) return 254;
  return static_cast<uint8_t>(std::min(cell->cost, Scalar(253)));
}

void TraversabilityMap::shiftTo(const Eigen::Matrix<Scalar, 2, 1>& robot_xy) {
  std::lock_guard<std::mutex> lock(mutex_);
  elevation_grid_->shiftToOrigin(robot_xy);
}

}  // namespace m20::terrain