#include "m20_slam_navigation/terrain/step_detector.hpp"

#include <algorithm>
#include <cmath>

namespace m20::terrain {

StepDetector::StepDetector(const TerrainParams& params)
    : params_(params) {}

void StepDetector::analyze(ElevationGrid& grid) {
  // 8-connected neighbourhood offsets
  const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

  for (int gy = 0; gy < grid.height(); ++gy) {
    for (int gx = 0; gx < grid.width(); ++gx) {
      ElevationCell* cell = grid.mutableCellAt(
          grid.gridToWorld(gx, gy).x(), grid.gridToWorld(gx, gy).y());
      if (!cell || cell->n_points < 1) continue;

      Scalar my_mean = cell->mean_z;
      Scalar max_pos_step = 0;
      Scalar max_neg_step = 0;

      // Compare with neighbours
      for (int k = 0; k < 8; ++k) {
        int nx = gx + dx[k];
        int ny = gy + dy[k];
        if (nx < 0 || nx >= grid.width() || ny < 0 || ny >= grid.height()) continue;

        const ElevationCell* neighbour = grid.cellAt(
            grid.gridToWorld(nx, ny).x(), grid.gridToWorld(nx, ny).y());
        if (!neighbour || neighbour->n_points < 1) continue;

        Scalar diff = neighbour->mean_z - my_mean;
        if (diff > max_pos_step) max_pos_step = diff;  // neighbour higher → positive step
        if (-diff > max_neg_step) max_neg_step = -diff;  // neighbour lower → negative step
      }

      cell->step_height = max_pos_step;

      // Flag untraversable if exceeds thresholds
      if (max_pos_step > params_.max_step_height) {
        cell->traversable = false;
        cell->cost += params_.step_weight * 255.0;
      } else if (max_neg_step > params_.max_step_depth) {
        cell->traversable = false;
        cell->cost += params_.step_weight * 255.0;
      } else {
        // Partial cost based on step height relative to max
        Scalar ratio = std::max(max_pos_step / params_.max_step_height,
                                max_neg_step / params_.max_step_depth);
        cell->cost += params_.step_weight * ratio * 200.0;
      }
    }
  }
}

}  // namespace m20::terrain