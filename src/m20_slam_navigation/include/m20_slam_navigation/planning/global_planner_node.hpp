#pragma once
/**
 * @file global_planner_node.hpp
 * @brief Global planner component: receives goal pose, outputs global path.
 *
 * Lifecycle:
 *   1. Receives traversability costmap from Terrain module.
 *   2. On new /goal_pose, runs Hybrid A* + spline smoothing.
 *   3. Publishes /plan as nav_msgs/Path.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/planning/hybrid_astar.hpp"
#include "m20_slam_navigation/planning/spline_optimizer.hpp"
#include "m20_slam_navigation/planning/motion_primitives.hpp"

#include <Eigen/Dense>

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace m20::planning {

using PlanCallback = std::function<void(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path)>;

class GlobalPlannerNode {
public:
  GlobalPlannerNode(const GlobalPlannerParams& planner_params,
                    const TerrainParams& terrain_params);

  /**
   * @brief Set current traversability costmap.
   */
  void setCostmap(const std::vector<uint8_t>& costmap,
                  int width, int height, Scalar resolution,
                  Scalar origin_x, Scalar origin_y);

  /**
   * @brief Request a new plan.
   *
   * @param start   Robot pose [x, y, θ]
   * @param goal    Goal pose [x, y, θ]
   * @return        true if plan found
   */
  bool plan(const Eigen::Matrix<Scalar, 3, 1>& start,
            const Eigen::Matrix<Scalar, 3, 1>& goal);

  /// Get latest plan
  std::vector<Eigen::Matrix<Scalar, 3, 1>> getPlan() const;

  /// Clear the current plan after a native cancel request.
  void clearPlan();

  /// Set plan callback
  void setPlanCallback(PlanCallback cb) { plan_cb_ = std::move(cb); }

private:
  GlobalPlannerParams                     planner_params_;
  TerrainParams                           terrain_params_;
  std::unique_ptr<HybridAStar>            hybrid_astar_;
  std::unique_ptr<SplineOptimizer>        spline_optimizer_;

  // Costmap cache
  std::vector<uint8_t>                    costmap_;
  int                                     costmap_width_{0};
  int                                     costmap_height_{0};
  Scalar                                  costmap_resolution_{0.1};
  Scalar                                  costmap_origin_x_{0};
  Scalar                                  costmap_origin_y_{0};

  // Latest plan
  std::vector<Eigen::Matrix<Scalar, 3, 1>> current_plan_;

  PlanCallback                            plan_cb_;
  mutable std::mutex                      mutex_;
};

}  // namespace m20::planning
