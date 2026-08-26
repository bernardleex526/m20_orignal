#include "m20_slam_navigation/terrain/roughness_analyzer.hpp"

#include <algorithm>
#include <cmath>

namespace m20::terrain {

RoughnessAnalyzer::RoughnessAnalyzer(const TerrainParams& params)
    : params_(params) {}

void RoughnessAnalyzer::analyze(ElevationGrid& grid) {
  for (int gy = 0; gy < grid.height(); ++gy) {
    for (int gx = 0; gx < grid.width(); ++gx) {
      ElevationCell* cell = grid.mutableCellAt(
          grid.gridToWorld(gx, gy).x(), grid.gridToWorld(gx, gy).y());
      if (!cell || cell->n_points < 2) continue;

      // σ_z = √(variance / (n − 1))
      Scalar sigma_z = std::sqrt(cell->var_z / static_cast<Scalar>(cell->n_points - 1));
      cell->roughness = sigma_z;

      // Cost: ramp from 0 to threshold
      Scalar cost = roughnessToCost(sigma_z, params_.roughness_threshold);
      cell->cost += params_.roughness_weight * cost * 200.0;
    }
  }
}

Scalar RoughnessAnalyzer::roughnessToCost(Scalar sigma_z, Scalar threshold) {
  if (sigma_z > threshold) return 1.0;
  if (threshold < 1e-10) return 0.0;
  return sigma_z / threshold;
}

}  // namespace m20::terrain