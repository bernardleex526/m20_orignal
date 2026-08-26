#include "m20_slam_navigation/planning/hybrid_astar.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

namespace m20::planning {

HybridAStar::HybridAStar(const GlobalPlannerParams& planner_params,
                         const TerrainParams& terrain_params)
    : planner_params_(planner_params)
    , terrain_params_(terrain_params)
    , num_heading_bins_(planner_params.num_heading_bins)
    , heading_bin_res_(2.0 * math::kPI / static_cast<Scalar>(planner_params.num_heading_bins)) {

  // Generate motion primitives
  primitives_ = MotionPrimitives::generateOmnidirectional(
      8, planner_params.primitive_length, planner_params.max_curvature, 3);
}

GridIndex HybridAStar::stateToGrid(Scalar x, Scalar y, Scalar theta) const {
  GridIndex idx;
  idx.ix = static_cast<int>(std::floor((x - dijkstra_origin_x_) / planner_params_.grid_resolution));
  idx.iy = static_cast<int>(std::floor((y - dijkstra_origin_y_) / planner_params_.grid_resolution));
  // Discretize theta
  int t = static_cast<int>(std::floor(math::normalize_angle(theta) / heading_bin_res_ + 0.5));
  idx.itheta = ((t % num_heading_bins_) + num_heading_bins_) % num_heading_bins_;
  return idx;
}

void HybridAStar::computeDijkstraHeuristic(
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    Scalar origin_x, Scalar origin_y,
    Scalar goal_x, Scalar goal_y) {

  dijkstra_costmap_.assign(width * height, std::numeric_limits<Scalar>::max());
  dijkstra_width_ = width;
  dijkstra_height_ = height;
  dijkstra_resolution_ = resolution;
  dijkstra_origin_x_ = origin_x;
  dijkstra_origin_y_ = origin_y;

  // BFS from goal outward (2D)
  using Node = std::pair<Scalar, int>;  // (cost, flat_index)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  int goal_ix = static_cast<int>(std::floor((goal_x - origin_x) / resolution));
  int goal_iy = static_cast<int>(std::floor((goal_y - origin_y) / resolution));
  int goal_idx = goal_iy * width + goal_ix;

  if (goal_ix >= 0 && goal_ix < width && goal_iy >= 0 && goal_iy < height) {
    dijkstra_costmap_[goal_idx] = 0;
    pq.push({0, goal_idx});
  }

  const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  const Scalar diag_cost = std::sqrt(Scalar(2)) * resolution;
  const Scalar straight_cost = resolution;

  while (!pq.empty()) {
    auto [cost, idx] = pq.top(); pq.pop();
    if (cost > dijkstra_costmap_[idx]) continue;

    int cx = idx % width;
    int cy = idx / width;

    for (int k = 0; k < 8; ++k) {
      int nx = cx + dx[k];
      int ny = cy + dy[k];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

      int nidx = ny * width + nx;
      if (costmap[nidx] >= 254) continue;  // lethal

      Scalar step_cost = (dx[k] != 0 && dy[k] != 0) ? diag_cost : straight_cost;
      // Add terrain cost
      Scalar terrain_penalty = costmap[nidx] / 255.0 * straight_cost;
      Scalar new_cost = cost + step_cost + terrain_penalty;

      if (new_cost < dijkstra_costmap_[nidx]) {
        dijkstra_costmap_[nidx] = new_cost;
        pq.push({new_cost, nidx});
      }
    }
  }
}

std::vector<Eigen::Matrix<Scalar, 3, 1>> HybridAStar::plan(
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    Scalar origin_x, Scalar origin_y,
    const Eigen::Matrix<Scalar, 3, 1>& start,
    const Eigen::Matrix<Scalar, 3, 1>& goal) {

  search_start_ = std::chrono::steady_clock::now();

  // Precompute Dijkstra heuristic (must have been called before, or redo)
  if (dijkstra_costmap_.empty() ||
      dijkstra_width_ != width ||
      dijkstra_height_ != height ||
      std::abs(dijkstra_resolution_ - resolution) > 1e-6) {
    computeDijkstraHeuristic(costmap, width, height, resolution, origin_x, origin_y,
                             goal.x(), goal.y());
  }

  // A* open list: min-heap by f = g + h
  using OpenNode = std::pair<Scalar, int>;  // (f_cost, index into closed_list)
  std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open;

  std::vector<HybridState> closed_list;
  std::unordered_map<GridIndex, int, GridIndex::Hash> state_to_idx;

  // Insert start
  {
    HybridState start_state;
    start_state.x = start.x();
    start_state.y = start.y();
    start_state.theta = start.z();
    start_state.g_cost = 0;
    start_state.h_cost = nonHoloHeuristic(start.x(), start.y(), start.theta(),
                                           goal.x(), goal.y(), goal.z());
    start_state.parent_idx = -1;
    start_state.closed = false;

    closed_list.push_back(start_state);
    GridIndex gidx = stateToGrid(start.x(), start.y(), start.theta());
    state_to_idx[gidx] = 0;

    open.push({start_state.f_cost(), 0});
  }

  int expansions = 0;
  int goal_closed_idx = -1;

  while (!open.empty() && expansions < planner_params_.max_expansions) {
    // Check time limit
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - search_start_).count() > planner_params_.time_limit_sec) {
      break;
    }

    auto [f_cost, idx] = open.top(); open.pop();
    HybridState& state = closed_list[idx];

    if (state.closed) continue;
    state.closed = true;
    expansions++;

    // Check goal
    Scalar dist_to_goal = std::sqrt(
        (state.x - goal.x()) * (state.x - goal.x()) +
        (state.y - goal.y()) * (state.y - goal.y()));

    if (dist_to_goal < planner_params_.primitive_length * 0.5) {
      // Close enough: create goal state
      HybridState goal_state;
      goal_state.x = goal.x();
      goal_state.y = goal.y();
      goal_state.theta = goal.z();
      goal_state.g_cost = state.g_cost + dist_to_goal;
      goal_state.h_cost = 0;
      goal_state.parent_idx = idx;
      goal_state.closed = true;

      goal_closed_idx = static_cast<int>(closed_list.size());
      closed_list.push_back(goal_state);
      break;
    }

    // Expand primitives
    for (int pi = 0; pi < static_cast<int>(primitives_.size()); ++pi) {
      const auto& mp = primitives_[pi];

      Scalar nx, ny, ntheta;
      MotionPrimitives::apply(mp, state.x, state.y, state.theta, nx, ny, ntheta);

      // Check collision
      if (isCollision(nx, ny, ntheta, costmap, width, height, resolution, origin_x, origin_y)) {
        continue;
      }

      // Compute costs
      Scalar g_cost = state.g_cost + mp.cost;

      // Terrain cost from costmap
      int gx = static_cast<int>(std::floor((nx - origin_x) / resolution));
      int gy = static_cast<int>(std::floor((ny - origin_y) / resolution));
      if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
        Scalar terrain_cost = costmap[gy * width + gx] / 255.0 * mp.cost;
        g_cost += terrain_cost * 2.0;  // weight terrain
      }

      Scalar h_cost = nonHoloHeuristic(nx, ny, ntheta,
                                        goal.x(), goal.y(), goal.z());
      Scalar d_cost = std::numeric_limits<Scalar>::max();
      if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
        d_cost = dijkstra_costmap_[gy * width + gx];
      }
      // Use max heuristic
      h_cost = std::max(h_cost, d_cost);

      GridIndex nidx = stateToGrid(nx, ny, ntheta);
      auto it = state_to_idx.find(nidx);

      if (it != state_to_idx.end()) {
        // Already visited: check if better
        HybridState& existing = closed_list[it->second];
        if (!existing.closed && g_cost < existing.g_cost) {
          existing.g_cost = g_cost;
          existing.h_cost = h_cost;
          existing.parent_idx = idx;
          existing.primitive_idx = pi;
          open.push({existing.f_cost(), it->second});
        }
      } else {
        HybridState new_state;
        new_state.x = nx; new_state.y = ny; new_state.theta = ntheta;
        new_state.g_cost = g_cost;
        new_state.h_cost = h_cost;
        new_state.parent_idx = idx;
        new_state.primitive_idx = pi;
        new_state.closed = false;

        int new_idx = static_cast<int>(closed_list.size());
        closed_list.push_back(new_state);
        state_to_idx[nidx] = new_idx;
        open.push({new_state.f_cost(), new_idx});
      }
    }
  }

  if (goal_closed_idx < 0) {
    return {};  // no path found
  }

  // Reconstruct path
  return reconstructPath(closed_list, goal_closed_idx);
}

Scalar HybridAStar::nonHoloHeuristic(
    Scalar x, Scalar y, Scalar theta,
    Scalar gx, Scalar gy, Scalar gtheta) const {

  // 2D Euclidean distance (relaxed — ignores heading constraint)
  Scalar dist = std::sqrt((x - gx) * (x - gx) + (y - gy) * (y - gy));

  // Heading penalty (minimum rotation to align with goal heading)
  Scalar heading_error = std::abs(math::normalize_angle(
      std::atan2(gy - y, gx - x) - theta));
  Scalar heading_penalty = std::min(heading_error, math::k2PI - heading_error) * 0.3;

  return dist + heading_penalty;
}

std::vector<Eigen::Matrix<Scalar, 3, 1>> HybridAStar::reconstructPath(
    const std::vector<HybridState>& closed_list, int goal_idx) const {

  std::vector<Eigen::Matrix<Scalar, 3, 1>> path;

  int idx = goal_idx;
  while (idx >= 0) {
    const auto& s = closed_list[idx];
    path.push_back(Eigen::Matrix<Scalar, 3, 1>(s.x, s.y, s.theta));
    idx = s.parent_idx;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

bool HybridAStar::isCollision(
    Scalar x, Scalar y, Scalar theta,
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    Scalar origin_x, Scalar origin_y) const {

  // Robot footprint: simplified circle of radius 0.5m
  constexpr Scalar kRobotRadius = 0.5;
  constexpr int kChecks = 8;

  for (int i = 0; i < kChecks; ++i) {
    Scalar angle = static_cast<Scalar>(i) * 2.0 * math::kPI / kChecks;
    Scalar cx = x + kRobotRadius * std::cos(angle);
    Scalar cy = y + kRobotRadius * std::sin(angle);

    int gx = static_cast<int>(std::floor((cx - origin_x) / resolution));
    int gy = static_cast<int>(std::floor((cy - origin_y) / resolution));

    if (gx < 0 || gx >= width || gy < 0 || gy >= height) return true;
    if (costmap[gy * width + gx] >= 254) return true;  // lethal or unknown
  }

  return false;
}

}  // namespace m20::planning