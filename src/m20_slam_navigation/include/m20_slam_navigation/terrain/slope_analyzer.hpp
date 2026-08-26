#pragma once
/**
 * @file slope_analyzer.hpp
 * @brief Local surface slope estimation via PCA on neighbouring points.
 *
 * For each occupied grid cell, estimates the local surface normal vector
 * n̂ by performing Principal Component Analysis (PCA) on the 3D coordinates
 * of points in the neighbourhood:
 *
 *   C = (1/N) · Σ_i (p_i − p̄)(p_i − p̄)ᵀ   (3×3 covariance)
 *
 * The smallest eigenvalue λ_3 of C corresponds to the surface normal
 * direction. The slope angle θ is the angle between n̂ and the vertical
 * axis [0, 0, 1]:
 *
 *   θ = acos(|n̂_z|)   where n̂_z = n̂ · [0, 0, 1]
 *
 * Cells with θ > MaxClimbAngle (30°) are penalized.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/terrain/elevation_grid.hpp"

#include <Eigen/Dense>

#include <memory>

namespace m20::terrain {

class SlopeAnalyzer {
public:
  explicit SlopeAnalyzer(const TerrainParams& params);

  /**
   * @brief Compute slope angles for all occupied cells.
   *
   * For each cell, gathers neighbouring points within `normal_estimation_radius`
   * and computes the surface normal via PCA.
   *
   * @param grid    Elevation grid (mutated: slope field filled)
   * @param cloud   Current point cloud for PCA (transformed to world)
   */
  void analyze(ElevationGrid& grid,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  /// Compute slope angle from surface normal
  static Scalar normalToSlope(const Eigen::Matrix<Scalar, 3, 1>& normal);

  /// Compute surface normal via PCA on a set of points
  static Eigen::Matrix<Scalar, 3, 1> estimateNormal(
      const std::vector<Eigen::Matrix<Scalar, 3, 1>>& points);

private:
  TerrainParams params_;
};

}  // namespace m20::terrain