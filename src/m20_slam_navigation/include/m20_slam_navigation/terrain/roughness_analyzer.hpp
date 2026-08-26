#pragma once
/**
 * @file roughness_analyzer.hpp
 * @brief Terrain roughness estimation from point height variance.
 *
 * Roughness is the standard deviation of point heights within a grid cell:
 *   σ_z = √( (1/N)·Σ_i (z_i − z̄)² )
 *
 * High roughness indicates uneven terrain (rubble, gravel, vegetation)
 * that may challenge quadruped foot placement.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/terrain/elevation_grid.hpp"

#include <memory>

namespace m20::terrain {

class RoughnessAnalyzer {
public:
  explicit RoughnessAnalyzer(const TerrainParams& params);

  /**
   * @brief Compute roughness σ_z for each occupied cell.
   *
   * The variance is already accumulated incrementally in ElevationCell::update().
   * This method finalizes σ_z and computes the roughness cost.
   *
   * @param grid  Elevation grid (mutated: roughness + cost updated)
   */
  void analyze(ElevationGrid& grid);

  /// Map roughness σ_z to cost [0, 1]
  static Scalar roughnessToCost(Scalar sigma_z, Scalar threshold);

private:
  TerrainParams params_;
};

}  // namespace m20::terrain