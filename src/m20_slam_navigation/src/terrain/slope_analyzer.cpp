#include "m20_slam_navigation/terrain/slope_analyzer.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace m20::terrain {

SlopeAnalyzer::SlopeAnalyzer(const TerrainParams& params)
    : params_(params) {}

void SlopeAnalyzer::analyze(
    ElevationGrid& grid,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

  if (!cloud || cloud->empty()) return;

  // For each occupied cell, gather neighbourhood points and compute normal
  Scalar search_radius_sq = params_.normal_estimation_radius * params_.normal_estimation_radius;

  for (int gy = 0; gy < grid.height(); ++gy) {
    for (int gx = 0; gx < grid.width(); ++gx) {
      ElevationCell* cell = grid.mutableCellAt(
          grid.gridToWorld(gx, gy).x(), grid.gridToWorld(gx, gy).y());
      if (!cell || cell->n_points < 3) {
        if (cell) {
          cell->slope = 0;
          cell->cost += 0;  // no penalty
        }
        continue;
      }

      // Gather points in neighbourhood
      Eigen::Matrix<Scalar, 2, 1> cell_center = grid.gridToWorld(gx, gy);
      std::vector<Eigen::Matrix<Scalar, 3, 1>> neighbourhood;

      for (const auto& pt : cloud->points) {
        Scalar dx = pt.x - cell_center.x();
        Scalar dy = pt.y - cell_center.y();
        if (dx * dx + dy * dy < search_radius_sq) {
          neighbourhood.push_back(Eigen::Matrix<Scalar, 3, 1>(pt.x, pt.y, pt.z));
        }
      }

      if (neighbourhood.size() >= 3) {
        Eigen::Matrix<Scalar, 3, 1> normal = estimateNormal(neighbourhood);
        Scalar slope_angle = normalToSlope(normal);

        cell->slope = slope_angle;

        // Cost: linear ramp from 0° to MaxClimbAngle
        if (slope_angle > params_.max_climb_angle) {
          cell->traversable = false;
          cell->cost += params_.slope_weight * 255.0;
        } else if (slope_angle > 0) {
          Scalar ratio = slope_angle / params_.max_climb_angle;
          cell->cost += params_.slope_weight * ratio * 200.0;
        }
      }
    }
  }
}

Scalar SlopeAnalyzer::normalToSlope(const Eigen::Matrix<Scalar, 3, 1>& normal) {
  // Slope angle = angle between normal and vertical [0, 0, 1]
  Scalar nz = std::abs(normal.z());
  nz = math::clamp(nz, Scalar(0), Scalar(1));
  return std::acos(nz);  // [0, π/2]
}

Eigen::Matrix<Scalar, 3, 1> SlopeAnalyzer::estimateNormal(
    const std::vector<Eigen::Matrix<Scalar, 3, 1>>& points) {

  if (points.size() < 3) {
    return Eigen::Matrix<Scalar, 3, 1>(0, 0, 1);  // default: flat
  }

  // Compute mean
  Eigen::Matrix<Scalar, 3, 1> mean{0, 0, 0};
  for (const auto& p : points) mean += p;
  mean /= static_cast<Scalar>(points.size());

  // Compute covariance (3×3)
  Eigen::Matrix<Scalar, 3, 3> cov = Eigen::Matrix<Scalar, 3, 3>::Zero();
  for (const auto& p : points) {
    Eigen::Matrix<Scalar, 3, 1> d = p - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<Scalar>(points.size());

  // PCA: smallest eigenvalue → normal direction
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 3, 3>> solver(cov);
  if (solver.info() != Eigen::Success) {
    return Eigen::Matrix<Scalar, 3, 1>(0, 0, 1);
  }

  // Smallest eigenvalue index = 0 (ascending order)
  Eigen::Matrix<Scalar, 3, 1> normal = solver.eigenvectors().col(0);

  // Ensure normal points upward (positive z)
  if (normal.z() < 0) normal = -normal;

  normal.normalize();
  return normal;
}

}  // namespace m20::terrain