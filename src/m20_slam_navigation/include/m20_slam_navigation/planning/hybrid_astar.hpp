#pragma once
/**
 * @file hybrid_astar.hpp
 * @brief Omnidirectional Hybrid A* global planner for quadruped robots.
 *
 * State space: [x, y, θ] where (x, y) are continuous coordinates and θ is the
 * continuous heading angle (discretized into N bins for search efficiency).
 *
 * Heuristic:
 *   h(state) = max( h_nonholonomic, h_dijkstra_obstacle )
 *
 * where:
 *   - h_nonholonomic = Dubins/Reeds-Shepp distance to goal (ignoring obstacles)
 *   - h_dijkstra_obstacle = 2D Dijkstra cost-to-go from goal to all cells
 *     (precomputed, accounts for obstacles)
 *
 * The max heuristic ensures both admissibility (never overestimates) and
 * efficiency (guides search around obstacles).
 *
 * After search, the raw path is smoothed via cubic spline optimization.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/planning/motion_primitives.hpp"

#include <Eigen/Dense>

#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

namespace m20::planning {

/// Continuous state for A* search
struct HybridState {
  Scalar x{0}, y{0}, theta{0};   ///< continuous pose
  Scalar g_cost{0};              ///< path cost from start
  Scalar h_cost{0};              ///< heuristic to goal
  Scalar f_cost() const { return g_cost + h_cost; }
  int    parent_idx{-1};         ///< index of parent in closed list
  int    primitive_idx{-1};      ///< which primitive was applied from parent
  bool   closed{false};

  bool operator<(const HybridState& o) const { return f_cost() > o.f_cost(); }  // min-heap
};

/// Grid index for lookup
struct GridIndex {
  int ix{0}, iy{0}, itheta{0};

  bool operator==(const GridIndex& o) const {
    return ix == o.ix && iy == o.iy && itheta == o.itheta;
  }

  struct Hash {
    std::size_t operator()(const GridIndex& k) const {
      return (static_cast<std::size_t>(k.ix) * 73856093) ^
             (static_cast<std::size_t>(k.iy) * 19349663) ^
             (static_cast<std::size_t>(k.itheta) * 83492791);
    }
  };
};

class HybridAStar {
public:
  HybridAStar(const GlobalPlannerParams& planner_params,
              const TerrainParams& terrain_params);

  /**
   * @brief Plan a path from start to goal.
   *
   * @param costmap       2D traversability costmap (row-major, [0, 255])
   * @param width         Costmap width [cells]
   * @param height        Costmap height [cells]
   * @param resolution    Costmap resolution [m/cell]
   * @param origin_x      Costmap origin x [m]
   * @param origin_y      Costmap origin y [m]
   * @param start         Start pose [x, y, θ]
   * @param goal          Goal pose [x, y, θ]
   * @return              Planned path (list of [x, y, θ] waypoints), empty on failure
   */
  std::vector<Eigen::Matrix<Scalar, 3, 1>> plan(
      const std::vector<uint8_t>& costmap,
      int width, int height, Scalar resolution,
      Scalar origin_x, Scalar origin_y,
      const Eigen::Matrix<Scalar, 3, 1>& start,
      const Eigen::Matrix<Scalar, 3, 1>& goal);

  /// Precompute Dijkstra heuristic from goal (call once per goal)
  void computeDijkstraHeuristic(const std::vector<uint8_t>& costmap,
                                int width, int height, Scalar resolution,
                                Scalar origin_x, Scalar origin_y,
                                Scalar goal_x, Scalar goal_y);

private:
  /// Convert continuous (x, y, θ) to discrete grid index
  GridIndex stateToGrid(Scalar x, Scalar y, Scalar theta) const;

  /// Non-holonomic heuristic (2D Euclidean distance with heading)
  Scalar nonHoloHeuristic(Scalar x, Scalar y, Scalar theta,
                          Scalar gx, Scalar gy, Scalar gtheta) const;

  /// Reconstruct path by following parent pointers
  std::vector<Eigen::Matrix<Scalar, 3, 1>> reconstructPath(
      const std::vector<HybridState>& closed_list, int goal_idx) const;

  /// Check if state is in collision
  bool isCollision(Scalar x, Scalar y, Scalar theta,
                   const std::vector<uint8_t>& costmap,
                   int width, int height, Scalar resolution,
                   Scalar origin_x, Scalar origin_y) const;

  GlobalPlannerParams           planner_params_;
  TerrainParams                 terrain_params_;
  std::vector<MotionPrimitive>  primitives_;
  int                           num_heading_bins_;
  Scalar                        heading_bin_res_;  ///< radians per bin

  // Dijkstra heuristic lookup
  std::vector<Scalar>           dijkstra_costmap_;
  int                           dijkstra_width_{0};
  int                           dijkstra_height_{0};
  Scalar                        dijkstra_resolution_{0};
  Scalar                        dijkstra_origin_x_{0};
  Scalar                        dijkstra_origin_y_{0};

  // Time limit tracking
  std::chrono::steady_clock::time_point search_start_;
};

}  // namespace m20::planning