#pragma once
/**
 * @file step_detector.hpp
 * @brief Positive and negative obstacle (step) detection.
 *
 * Step height is computed as the maximum positive height difference between
 * a cell and its immediate neighbours (8-connected):
 *
 *   Δh_pos(i,j) = max_{k∈N(i,j)} (z_k − z_{ij})
 *   Δh_neg(i,j) = max_{k∈N(i,j)} (z_{ij} − z_k)
 *
 * Steps exceeding MaxStepHeight (0.20m positive) or MaxStepDepth (0.15m
 * negative) are flagged as untraversable. This is critical for quadruped
 * robots that can step over small obstacles but cannot climb vertical walls
 * or jump down large drops.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/terrain/elevation_grid.hpp"

#include <memory>

namespace m20::terrain {

class StepDetector {
public:
  explicit StepDetector(const TerrainParams& params);

  /**
   * @brief Detect steps in elevation grid.
   *
   * Sets traversable = false and cost = 255 for cells where step exceeds
   * thresholds.
   *
   * @param grid  Elevation grid (mutated)
   */
  void analyze(ElevationGrid& grid);

private:
  TerrainParams params_;
};

}  // namespace m20::terrain