#pragma once
/**
 * @file spline_optimizer.hpp
 * @brief Cubic spline trajectory smoothing for Hybrid A* paths.
 *
 * The raw Hybrid A* path consists of piecewise-linear segments with abrupt
 * direction changes. This optimizer fits a cubic B-spline / parametric cubic
 * spline to the waypoints while enforcing:
 *
 *   - Smoothness: minimize Σ ‖p''(s)‖² ds  (integrated squared curvature)
 *   - Obstacle clearance: penalty for deviation from original path
 *   - Boundary curvature: κ(s) ≤ κ_max at all points
 *
 * Formulation (unconstrained optimization):
 *   min Σ_i { ‖p_i − p_i⁰‖² + λ·‖p_i''‖² }
 *
 * where p_i are the smoothed waypoints, p_i⁰ are the original A* waypoints,
 * and λ is the smoothness weight.
 *
 * Curvature at waypoint i is approximated via finite differences:
 *   κ_i ≈ ‖p_{i-1} − 2p_i + p_{i+1}‖ / ‖p_{i+1} − p_{i-1}‖²
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Dense>

#include <vector>

namespace m20::planning {

class SplineOptimizer {
public:
  explicit SplineOptimizer(const GlobalPlannerParams& params);

  /**
   * @brief Smooth a sequence of waypoints using cubic spline optimization.
   *
   * @param path_in    Input waypoints [x, y, θ] from Hybrid A*
   * @param costmap    2D costmap for obstacle checking
   * @param width      Costmap width
   * @param height     Costmap height
   * @param resolution Costmap resolution
   * @param origin_x   Costmap origin x
   * @param origin_y   Costmap origin y
   * @return           Smoothed waypoints
   */
  std::vector<Eigen::Matrix<Scalar, 3, 1>> smooth(
      const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path_in,
      const std::vector<uint8_t>& costmap,
      int width, int height, Scalar resolution,
      Scalar origin_x, Scalar origin_y);

private:
  /// Compute curvature at a waypoint via finite differences
  static Scalar computeCurvature(const Eigen::Matrix<Scalar, 3, 1>& prev,
                                 const Eigen::Matrix<Scalar, 3, 1>& curr,
                                 const Eigen::Matrix<Scalar, 3, 1>& next);

  /// Check whether the robot footprint at a pose is collision-free.
  bool isCollisionFree(Scalar x, Scalar y, Scalar theta,
                       const std::vector<uint8_t>& costmap,
                       int width, int height, Scalar resolution,
                       Scalar origin_x, Scalar origin_y) const;

  /// Validate every pose and the swept footprint between adjacent poses.
  bool isPathCollisionFree(
      const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path,
      const std::vector<uint8_t>& costmap,
      int width, int height, Scalar resolution,
      Scalar origin_x, Scalar origin_y) const;

  GlobalPlannerParams params_;
};

}  // namespace m20::planning
