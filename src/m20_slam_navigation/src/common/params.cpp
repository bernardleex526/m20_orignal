#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Eigenvalues>

namespace m20 {

// Default construction provides compile-time constants.
// Runtime parameter loading from YAML is handled by ROS 2 node wrappers
// using rclcpp parameter callbacks or external YAML parsing.

void VoxelEntry::addPoint(const Eigen::Matrix<Scalar, 3, 1>& p) {
  plane_valid = false;
  // Welford's online algorithm for incremental mean and covariance
  point_count++;
  Scalar n = static_cast<Scalar>(point_count);

  if (point_count == 1) {
    centroid = p;
    covariance.setZero();
  } else {
    Eigen::Matrix<Scalar, 3, 1> delta = p - centroid;
    centroid += delta / n;
    Eigen::Matrix<Scalar, 3, 1> delta2 = p - centroid;
    covariance += delta * delta2.transpose();
    // Note: covariance becomes Σ = accumulated / (n − 1) when queried
  }
}

void VoxelEntry::updatePlane(Scalar eigenvalue_ratio) {
  plane_valid = false;
  if (point_count < 5U) return;
  const auto normalized_covariance = covariance / static_cast<Scalar>(point_count - 1U);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 3, 3>> solver(normalized_covariance);
  if (solver.info() != Eigen::Success) return;
  const auto eigenvalues = solver.eigenvalues();
  if (!eigenvalues.allFinite() || eigenvalues.z() <= Scalar(1e-10) ||
      eigenvalues.x() > eigenvalue_ratio * eigenvalues.y()) {
    return;
  }
  plane_normal = solver.eigenvectors().col(0);
  plane_offset = -plane_normal.dot(centroid);
  plane_valid = true;
}

void ElevationCell::update(Scalar z) {
  // Welford for elevation
  n_points++;
  Scalar n = static_cast<Scalar>(n_points);
  Scalar delta = z - mean_z;
  mean_z += delta / n;
  Scalar delta2 = z - mean_z;
  var_z += delta * delta2;  // accumulated sum of squares

  if (z < min_z) min_z = z;
  if (z > max_z) max_z = z;
}

}  // namespace m20
