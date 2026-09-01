#include "m20_slam_navigation/terrain/traversability_map.hpp"

#include <algorithm>
#include <cmath>

namespace m20::terrain {

TraversabilityMap::TraversabilityMap(const TerrainParams& terrain_params)
    : params_(terrain_params)
    , elevation_grid_(std::make_unique<ElevationGrid>(
        terrain_params, terrain_params.map_length > 0.0 ? terrain_params.map_length : 100.0))
    , slope_analyzer_(std::make_unique<SlopeAnalyzer>(terrain_params))
    , roughness_analyzer_(std::make_unique<RoughnessAnalyzer>(terrain_params))
    , step_detector_(std::make_unique<StepDetector>(terrain_params)) {
}

void TraversabilityMap::update(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const SE3Pose& T_world_lidar) {

  std::lock_guard<std::mutex> lock(mutex_);

  if (!cloud || cloud->empty()) return;

  // Keep the rolling grid centered before inserting the current scan so a
  // moved robot cannot silently discard points against the previous origin.
  Eigen::Matrix<Scalar, 2, 1> robot_xy(T_world_lidar.t.x(), T_world_lidar.t.y());
  elevation_grid_->shiftToOrigin(robot_xy);

  // Step 1: Update elevation grid
  elevation_grid_->update(cloud, T_world_lidar);

  auto cloud_world = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud_world->reserve(cloud->size());
  for (const auto& point : cloud->points) {
    const Eigen::Matrix<Scalar, 3, 1> transformed = T_world_lidar.transformPoint(
        {point.x, point.y, point.z});
    pcl::PointXYZI output;
    output.x = static_cast<float>(transformed.x());
    output.y = static_cast<float>(transformed.y());
    output.z = static_cast<float>(transformed.z());
    output.intensity = point.intensity;
    cloud_world->push_back(output);
  }

  // Recompute derived costs from the current elevation statistics. Without
  // this reset, repeated scans would add the same cost until every cell became
  // lethal even when the terrain itself had not changed.
  for (int gy = 0; gy < elevation_grid_->height(); ++gy) {
    for (int gx = 0; gx < elevation_grid_->width(); ++gx) {
      const auto center = elevation_grid_->gridToWorld(gx, gy);
      if (auto* cell = elevation_grid_->mutableCellAt(center.x(), center.y())) {
        cell->slope = 0.0;
        cell->roughness = 0.0;
        cell->step_height = 0.0;
        cell->traversable = true;
        cell->cost = 0.0;
      }
    }
  }

  // Step 2-4: Analyze terrain in the same world frame as the grid.
  slope_analyzer_->analyze(*elevation_grid_, cloud_world);
  roughness_analyzer_->analyze(*elevation_grid_);
  step_detector_->analyze(*elevation_grid_);
}

std::vector<uint8_t> TraversabilityMap::exportCostmap() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<uint8_t> costmap(elevation_grid_->cells().size(), 0);

  for (std::size_t i = 0; i < elevation_grid_->cells().size(); ++i) {
    const auto& cell = elevation_grid_->cells()[i];

    if (cell.n_points == 0) {
      costmap[i] = params_.treat_nan_as_stiff
          ? 255
          : static_cast<uint8_t>(std::clamp(
              params_.traversal_missing_cost, Scalar(0), Scalar(253)));
    } else if (!cell.traversable) {
      costmap[i] = 254;  // lethal
    } else {
      // Clamp cost to [0, 253]
      Scalar cost = params_.traversal_cost_enable ? cell.cost : Scalar(0);
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

Scalar TraversabilityMap::originX() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return elevation_grid_->origin().x();
}

Scalar TraversabilityMap::originY() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return elevation_grid_->origin().y();
}

uint8_t TraversabilityMap::costAt(Scalar x, Scalar y) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const ElevationCell* cell = elevation_grid_->cellAt(x, y);
  if (!cell || cell->n_points == 0) {
    return params_.treat_nan_as_stiff
        ? 255
        : static_cast<uint8_t>(std::clamp(
            params_.traversal_missing_cost, Scalar(0), Scalar(253)));
  }
  if (!cell->traversable) return 254;
  return static_cast<uint8_t>(std::min(
      params_.traversal_cost_enable ? cell->cost : Scalar(0), Scalar(253)));
}

void TraversabilityMap::shiftTo(const Eigen::Matrix<Scalar, 2, 1>& robot_xy) {
  std::lock_guard<std::mutex> lock(mutex_);
  elevation_grid_->shiftToOrigin(robot_xy);
}

}  // namespace m20::terrain
