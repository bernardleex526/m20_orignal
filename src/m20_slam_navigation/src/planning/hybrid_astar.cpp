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
    , primitives_(MotionPrimitives::generateOmnidirectional(
        planner_params.num_directions,
        planner_params.step_size > 0.0
          ? planner_params.step_size : planner_params.primitive_length,
        planner_params.max_curvature,
        planner_params.num_steerind > 0
          ? planner_params.num_steerind : planner_params.num_rotations,
        planner_params.max_steer * planner_params.sample_interval))
    , num_heading_bins_(std::max(planner_params.num_heading_bins, 1))
    , heading_bin_res_(2.0 * math::kPI / static_cast<Scalar>(num_heading_bins_))
    , state_grid_resolution_(planner_params.grid_resolution) {}

GridIndex HybridAStar::stateToGrid(Scalar x, Scalar y, Scalar theta) const {
  GridIndex idx;
  const Scalar resolution = state_grid_resolution_ > 0.0
      ? state_grid_resolution_ : planner_params_.grid_resolution;
  idx.ix = static_cast<int>(std::floor((x - dijkstra_origin_x_) / resolution));
  idx.iy = static_cast<int>(std::floor((y - dijkstra_origin_y_) / resolution));
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
  dijkstra_goal_x_ = goal_x;
  dijkstra_goal_y_ = goal_y;

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
      Scalar terrain_penalty = costmap[nidx] / 255.0 * straight_cost *
                               planner_params_.weight_b;
      Scalar new_cost = cost + step_cost * planner_params_.weight_a + terrain_penalty;

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
  if (resolution > 0.0) state_grid_resolution_ = resolution;

  // Precompute Dijkstra heuristic (must have been called before, or redo)
  if (dijkstra_costmap_.empty() ||
      dijkstra_width_ != width ||
      dijkstra_height_ != height ||
      std::abs(dijkstra_resolution_ - resolution) > 1e-6 ||
      std::abs(dijkstra_origin_x_ - origin_x) > 1e-6 ||
      std::abs(dijkstra_origin_y_ - origin_y) > 1e-6 ||
      std::abs(dijkstra_goal_x_ - goal.x()) > 1e-6 ||
      std::abs(dijkstra_goal_y_ - goal.y()) > 1e-6) {
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
    start_state.h_cost = nonHoloHeuristic(start.x(), start.y(), start.z(),
                                           goal.x(), goal.y(), goal.z());
    start_state.parent_idx = -1;
    start_state.closed = false;

    closed_list.push_back(start_state);
    GridIndex gidx = stateToGrid(start.x(), start.y(), start.z());
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
    if (closed_list[idx].closed) continue;
    closed_list[idx].closed = true;
    // Keep a value copy: expanding a primitive can grow closed_list and
    // invalidate references to its storage.
    const HybridState state = closed_list[idx];
    expansions++;

    // Check goal
    Scalar dist_to_goal = std::sqrt(
        (state.x - goal.x()) * (state.x - goal.x()) +
        (state.y - goal.y()) * (state.y - goal.y()));

    if (dist_to_goal <= std::max(
            planner_params_.goal_dis, planner_params_.xy_tolerance)) {
      if (!isSegmentCollisionFree(
              state.x, state.y, state.theta,
              goal.x(), goal.y(), goal.z(),
              costmap, width, height, resolution, origin_x, origin_y)) {
        continue;
      }
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

      // Check the swept robot footprint, not only the primitive endpoint.
      if (!isSegmentCollisionFree(
              state.x, state.y, state.theta, nx, ny, ntheta,
              costmap, width, height, resolution, origin_x, origin_y)) {
        continue;
      }

      // Compute costs
      const Scalar translation_length = std::hypot(mp.dx, mp.dy);
      const Scalar heading_change = std::abs(mp.dtheta);
      const bool backwards = mp.dx < -1e-6;
      const bool changed_steer = state.primitive_idx >= 0 &&
          std::abs(mp.dtheta - primitives_[state.primitive_idx].dtheta) > 1e-6;
      Scalar g_cost = state.g_cost + planner_params_.weight_a * translation_length
                    + planner_params_.weight_heading * heading_change
                    + planner_params_.cost_steer * heading_change
                    + (changed_steer ? planner_params_.cost_steerchange : 0.0)
                    + (backwards ? planner_params_.cost_gear + planner_params_.cost_backward : 0.0);

      // Terrain cost from costmap
      int gx = static_cast<int>(std::floor((nx - origin_x) / resolution));
      int gy = static_cast<int>(std::floor((ny - origin_y) / resolution));
      if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
        Scalar terrain_cost = costmap[gy * width + gx] / 255.0 *
                              translation_length * planner_params_.weight_b;
        // cost_reduce is a native terrain-cost scale, not a discount on the
        // accumulated path cost.  Applying it to g_cost made a longer path
        // cheaper whenever it crossed any non-zero cell.
        g_cost += terrain_cost * std::max(planner_params_.cost_reduce, Scalar(0.01));
      }

      Scalar h_cost = nonHoloHeuristic(nx, ny, ntheta,
                                        goal.x(), goal.y(), goal.z());
      Scalar d_cost = std::numeric_limits<Scalar>::max();
      if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
        d_cost = dijkstra_costmap_[gy * width + gx];
      }
      // Use max heuristic
      h_cost = std::max(h_cost * std::max(planner_params_.heuristic_weight, Scalar(0.0)),
                        d_cost);

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

  return planner_params_.weight_a * dist + planner_params_.weight_heading * heading_penalty;
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

  const Scalar half_length = std::max(planner_params_.body_length * 0.5, Scalar(0.01));
  const Scalar half_width = std::max(planner_params_.body_width * 0.5, Scalar(0.01));
  const int length_steps = std::max(1, static_cast<int>(std::ceil(half_length / resolution)));
  const int width_steps = std::max(1, static_cast<int>(std::ceil(half_width / resolution)));
  const Scalar c = std::cos(theta);
  const Scalar s = std::sin(theta);

  for (int il = -length_steps; il <= length_steps; ++il) {
    for (int iw = -width_steps; iw <= width_steps; ++iw) {
      const Scalar local_x = il * resolution;
      const Scalar local_y = iw * resolution;
      const Scalar cx = x + c * local_x - s * local_y;
      const Scalar cy = y + s * local_x + c * local_y;
      const int gx = static_cast<int>(std::floor((cx - origin_x) / resolution));
      const int gy = static_cast<int>(std::floor((cy - origin_y) / resolution));
      if (gx < 0 || gx >= width || gy < 0 || gy >= height) return true;
      if (costmap[gy * width + gx] >= 254) return true;
    }
  }

  return false;
}

bool HybridAStar::isSegmentCollisionFree(
    Scalar x0, Scalar y0, Scalar theta0,
    Scalar x1, Scalar y1, Scalar theta1,
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    Scalar origin_x, Scalar origin_y) const {

  const Scalar distance = std::hypot(x1 - x0, y1 - y0);
  const Scalar angular_distance = std::abs(math::normalize_angle(theta1 - theta0));
  const Scalar linear_interval = std::max(
      std::min(planner_params_.sample_interval, resolution * Scalar(0.5)),
      Scalar(0.01));
  const Scalar angular_interval = std::max(heading_bin_res_ * Scalar(0.5), Scalar(0.05));
  const int samples = std::max(
      1, static_cast<int>(std::ceil(std::max(
          distance / linear_interval, angular_distance / angular_interval))));
  const Scalar delta_theta = math::normalize_angle(theta1 - theta0);

  for (int i = 0; i <= samples; ++i) {
    const Scalar ratio = static_cast<Scalar>(i) / static_cast<Scalar>(samples);
    const Scalar x = x0 + ratio * (x1 - x0);
    const Scalar y = y0 + ratio * (y1 - y0);
    const Scalar theta = math::normalize_angle(theta0 + ratio * delta_theta);
    if (isCollision(x, y, theta, costmap, width, height, resolution,
                    origin_x, origin_y)) {
      return false;
    }
  }
  return true;
}

}  // namespace m20::planning
