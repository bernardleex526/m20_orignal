#include "m20_slam_navigation/planning/global_planner_node.hpp"

namespace m20::planning {

GlobalPlannerNode::GlobalPlannerNode(const GlobalPlannerParams& planner_params,
                                     const TerrainParams& terrain_params)
    : planner_params_(planner_params)
    , terrain_params_(terrain_params)
    , hybrid_astar_(std::make_unique<HybridAStar>(planner_params, terrain_params))
    , spline_optimizer_(std::make_unique<SplineOptimizer>(planner_params)) {
}

void GlobalPlannerNode::setCostmap(
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    Scalar origin_x, Scalar origin_y) {

  std::lock_guard<std::mutex> lock(mutex_);
  costmap_ = costmap;
  costmap_width_ = width;
  costmap_height_ = height;
  costmap_resolution_ = resolution;
  costmap_origin_x_ = origin_x;
  costmap_origin_y_ = origin_y;
}

bool GlobalPlannerNode::plan(
    const Eigen::Matrix<Scalar, 3, 1>& start,
    const Eigen::Matrix<Scalar, 3, 1>& goal) {

  std::lock_guard<std::mutex> lock(mutex_);

  if (costmap_.empty()) return false;

  // Hybrid A* search
  auto raw_path = hybrid_astar_->plan(
      costmap_, costmap_width_, costmap_height_, costmap_resolution_,
      costmap_origin_x_, costmap_origin_y_, start, goal);

  if (raw_path.empty()) {
    current_plan_.clear();
    return false;
  }

  // Spline smoothing
  current_plan_ = spline_optimizer_->smooth(
      raw_path, costmap_, costmap_width_, costmap_height_,
      costmap_resolution_, costmap_origin_x_, costmap_origin_y_);

  // Notify
  if (plan_cb_) {
    plan_cb_(current_plan_);
  }

  return true;
}

std::vector<Eigen::Matrix<Scalar, 3, 1>> GlobalPlannerNode::getPlan() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_plan_;
}

void GlobalPlannerNode::clearPlan() {
  std::lock_guard<std::mutex> lock(mutex_);
  current_plan_.clear();
}

}  // namespace m20::planning
