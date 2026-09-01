#include "m20_slam_navigation/planning/spline_optimizer.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>

namespace m20::planning {

SplineOptimizer::SplineOptimizer(const GlobalPlannerParams& params)
    : params_(params) {}

std::vector<Eigen::Matrix<Scalar, 3, 1>> SplineOptimizer::smooth(
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path_in,
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    Scalar origin_x, Scalar origin_y) {

  if (path_in.size() < 3 || !params_.enable_smoothing) return path_in;

  const std::size_t N = path_in.size();
  const Scalar lambda = params_.spline_smoothness_weight;

  // We optimize x and y separately as two 1D cubic spline smoothing problems
  // For each coordinate: min Σ_i (p_i − pᵢ⁰)² + λ·Σ_i (p_{i−1} − 2p_i + p_{i+1})²
  //
  // This leads to a sparse linear system:
  //   (I + λ·DᵀD)·p = p⁰
  // where D is the second-order difference matrix.

  // Build DᵀD (tridiagonal + corners, (N−2)×N sparse)
  // D has rows like [1, −2, 1] (second derivative via finite differences)
  // DᵀD is pentadiagonal:
  //   [ 1  −2   1   0   ... ]
  //   [−2   5  −4   1   ... ]
  //   [ 1  −4   6  −4   1  ...]
  //   [ 0   1  −4   6  −4  ...]
  //   ...

  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> A =
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(N, N);

  // Add smoothness penalty: λ·DᵀD
  for (std::size_t i = 1; i < N - 1; ++i) {
    // Row i of D (offset by 1 since D has N-2 rows)
    // D(i-1, i-1)=1, D(i-1, i)=-2, D(i-1, i+1)=1
    // DᵀD contributions:
    A(i-1, i-1) += lambda * 1.0;
    A(i-1, i)   -= lambda * 2.0;
    A(i-1, i+1) += lambda * 1.0;

    A(i, i-1)   -= lambda * 2.0;
    A(i, i)     += lambda * 4.0;
    A(i, i+1)   -= lambda * 2.0;

    A(i+1, i-1) += lambda * 1.0;
    A(i+1, i)   -= lambda * 2.0;
    A(i+1, i+1) += lambda * 1.0;
  }

  // Solve for x and y
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> bx(N), by(N);
  for (std::size_t i = 0; i < N; ++i) {
    bx(i) = path_in[i].x();
    by(i) = path_in[i].y();
  }

  // LDLT solver
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sx = A.ldlt().solve(bx);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sy = A.ldlt().solve(by);

  // Build a candidate path. The complete swept footprint is validated below;
  // mixing isolated original and smoothed points can itself create shortcuts.
  std::vector<Eigen::Matrix<Scalar, 3, 1>> smoothed;
  smoothed.reserve(N);

  for (std::size_t i = 0; i < N; ++i) {
    // Compute curvature
    Scalar kappa = 0;
    if (i > 0 && i < N - 1) {
      Eigen::Matrix<Scalar, 3, 1> prev(sx(i-1), sy(i-1), 0);
      Eigen::Matrix<Scalar, 3, 1> curr(sx(i), sy(i), 0);
      Eigen::Matrix<Scalar, 3, 1> next(sx(i+1), sy(i+1), 0);
      kappa = computeCurvature(prev, curr, next);
    }

    // Heading from tangent
    Scalar theta = 0;
    if (i < N - 1) {
      theta = std::atan2(sy(i+1) - sy(i), sx(i+1) - sx(i));
    } else if (i > 0) {
      theta = std::atan2(sy(i) - sy(i-1), sx(i) - sx(i-1));
    }

    // If curvature exceeds max, fallback
    if (kappa > params_.max_curvature) {
      smoothed.push_back(path_in[i]);
    } else {
      smoothed.push_back(Eigen::Matrix<Scalar, 3, 1>(sx(i), sy(i), theta));
    }
  }

  // Native smoothing keeps the boundary poses fixed.  The old unconstrained
  // solve could move the start and goal away from the requested poses, which
  // made /path_Astar disagree with the goal service contract.
  if (!smoothed.empty()) {
    smoothed.front() = path_in.front();
    smoothed.back() = path_in.back();
  }

  return isPathCollisionFree(smoothed, costmap, width, height, resolution,
                             origin_x, origin_y) ? smoothed : path_in;
}

Scalar SplineOptimizer::computeCurvature(
    const Eigen::Matrix<Scalar, 3, 1>& prev,
    const Eigen::Matrix<Scalar, 3, 1>& curr,
    const Eigen::Matrix<Scalar, 3, 1>& next) {

  // κ = ‖p' × p''‖ / ‖p'‖³
  // Using finite differences:
  // p' ≈ (next − prev) / (2h)
  // p'' ≈ (prev − 2curr + next) / h²

  Eigen::Matrix<Scalar, 2, 1> dp((next.x() - prev.x()) / 2.0,
                                  (next.y() - prev.y()) / 2.0);
  Eigen::Matrix<Scalar, 2, 1> ddp(prev.x() - 2.0 * curr.x() + next.x(),
                                   prev.y() - 2.0 * curr.y() + next.y());

  Scalar dp_norm = dp.norm();
  if (dp_norm < 1e-6) return 0;

  // Cross product in 2D: |x1*y2 − y1*x2|
  Scalar cross = std::abs(dp.x() * ddp.y() - dp.y() * ddp.x());
  return cross / (dp_norm * dp_norm * dp_norm);
}

bool SplineOptimizer::isCollisionFree(
    Scalar x, Scalar y, Scalar theta,
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    Scalar origin_x, Scalar origin_y) const {

  const Scalar half_length = std::max(params_.body_length * Scalar(0.5), Scalar(0.01));
  const Scalar half_width = std::max(params_.body_width * Scalar(0.5), Scalar(0.01));
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
      if (gx < 0 || gx >= width || gy < 0 || gy >= height ||
          costmap[gy * width + gx] >= 254) {
        return false;
      }
    }
  }
  return true;
}

bool SplineOptimizer::isPathCollisionFree(
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path,
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    Scalar origin_x, Scalar origin_y) const {

  if (path.empty()) return false;
  const Scalar interval = std::max(
      std::min(params_.sample_interval, resolution * Scalar(0.5)), Scalar(0.01));

  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!isCollisionFree(path[i].x(), path[i].y(), path[i].z(), costmap,
                         width, height, resolution, origin_x, origin_y)) {
      return false;
    }
    if (i == 0) continue;

    const auto& previous = path[i - 1];
    const auto& current = path[i];
    const Scalar distance = std::hypot(
        current.x() - previous.x(), current.y() - previous.y());
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / interval)));
    const Scalar delta_theta = math::normalize_angle(current.z() - previous.z());
    for (int sample = 1; sample < samples; ++sample) {
      const Scalar ratio = static_cast<Scalar>(sample) / static_cast<Scalar>(samples);
      if (!isCollisionFree(
              previous.x() + ratio * (current.x() - previous.x()),
              previous.y() + ratio * (current.y() - previous.y()),
              math::normalize_angle(previous.z() + ratio * delta_theta),
              costmap, width, height, resolution, origin_x, origin_y)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace m20::planning
