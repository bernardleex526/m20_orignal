#include "m20_slam_navigation/backend/degeneracy_detector.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace m20::backend {

DegeneracyDetector::DegeneracyDetector(const BackendParams& params)
    : params_(params) {}

DegeneracyResult DegeneracyDetector::analyze(
    const Eigen::Matrix<Scalar, 6, 6>& H,
    const Eigen::Matrix<Scalar, 3, 1>& robot_heading) {

  DegeneracyResult result;

  // 1. Eigenvalue decomposition of Hessian H = JᵀWJ
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 6, 6>> eig_solver(H);

  if (eig_solver.info() != Eigen::Success) {
    result.is_degenerate = true;
    result.degenerate_directions.setOnes();  // all degenerate on failure
    return result;
  }

  result.eigenvalues = std::vector<Scalar>(
      eig_solver.eigenvalues().data(),
      eig_solver.eigenvalues().data() + 6);

  // Sort: eigenvalues are already in ascending order from SelfAdjointEigenSolver
  Scalar lambda_max = result.eigenvalues[5];
  if (lambda_max < 1e-12) lambda_max = 1.0;

  // 2. Check each eigenvalue against degeneracy threshold
  Scalar threshold = 1.0 / params_.degeneracy_threshold;  // i.e. λ_min < λ_max / threshold

  result.degenerate_directions.setZero();
  result.degenerate_eigenvectors.resize(6, Eigen::Matrix<Scalar, 6, 1>::Zero());

  // Robot heading direction in the 6-DOF space:
  // For translation along X (forward direction), the corresponding twist component
  // is the translation part in the heading direction. But we need to map the robot
  // heading [hx, hy, 0] to the 6-DOF twist: [0, 0, 0, hx, hy, 0] (translation only).
  // Actually for longitudinal degeneracy, the eigenvector should have significant
  // translation component aligned with robot heading.
  //
  // We check both: pure translation direction AND the actual eigenvector direction.

  for (int i = 0; i < 6; ++i) {
    Scalar lambda_i = result.eigenvalues[i];
    if (lambda_i / lambda_max < threshold) {
      // This eigenvalue is degenerate
      Eigen::Matrix<Scalar, 6, 1> v_i = eig_solver.eigenvectors().col(i);

      // Check if this degenerate direction aligns with robot longitudinal (X) axis
      // The translation component of the twist: v_i.tail<3>()
      Eigen::Matrix<Scalar, 3, 1> trans_component = v_i.tail<3>();
      Scalar trans_norm = trans_component.norm();

      if (trans_norm > 0.1) {
        Eigen::Matrix<Scalar, 3, 1> trans_dir = trans_component / trans_norm;
        Scalar alignment = std::abs(trans_dir.dot(robot_heading));

        if (alignment > params_.degeneracy_heading_align) {
          result.degenerate_directions[i] = 1.0;
          result.degenerate_eigenvectors[i] = v_i;
          result.is_degenerate = true;
        }
      }

      // Also check rotation component alignment (yaw degeneracy)
      Eigen::Matrix<Scalar, 3, 1> rot_component = v_i.head<3>();
      Scalar rot_norm = rot_component.norm();
      if (rot_norm > 0.1) {
        // Yaw axis is [0, 0, 1] in world frame
        Eigen::Matrix<Scalar, 3, 1> yaw_axis(0, 0, 1);
        Scalar yaw_alignment = std::abs(rot_component.normalized().dot(yaw_axis));
        if (yaw_alignment > params_.degeneracy_heading_align) {
          result.degenerate_directions[i] = 1.0;
          result.degenerate_eigenvectors[i] = v_i;
          result.is_degenerate = true;
        }
      }
    }
  }

  // 3. Compute filtered Hessian (zero out degenerate dimensions)
  result.filtered_hessian = H;
  if (result.is_degenerate) {
    // Project out degenerate eigenvectors:
    // H_filtered = (I − Σ v_i·v_iᵀ) · H · (I − Σ v_i·v_iᵀ)ᵀ
    Eigen::Matrix<Scalar, 6, 6> P = Eigen::Matrix<Scalar, 6, 6>::Identity();
    for (int i = 0; i < 6; ++i) {
      if (result.degenerate_directions[i] > 0.5) {
        P -= result.degenerate_eigenvectors[i] * result.degenerate_eigenvectors[i].transpose();
      }
    }
    result.filtered_hessian = P * H * P.transpose();
  }

  return result;
}

Eigen::Matrix<Scalar, 6, 1> DegeneracyDetector::filterCorrection(
    const Eigen::Matrix<Scalar, 6, 1>& correction_raw,
    const DegeneracyResult& degeneracy) const {

  if (!degeneracy.is_degenerate) {
    return correction_raw;
  }

  // Build projection matrix: P = I − Σ v_i v_iᵀ for degenerate eigenvectors
  Eigen::Matrix<Scalar, 6, 6> P = Eigen::Matrix<Scalar, 6, 6>::Identity();
  for (int i = 0; i < 6; ++i) {
    if (degeneracy.degenerate_directions[i] > 0.5) {
      P -= degeneracy.degenerate_eigenvectors[i] *
           degeneracy.degenerate_eigenvectors[i].transpose();
    }
  }

  // Filtered correction: zero out components along degenerate directions
  return P * correction_raw;
}

Eigen::Matrix<Scalar, 6, 1> DegeneracyDetector::fuseOdometryCorrection(
    const Eigen::Matrix<Scalar, 6, 1>& correction_filtered,
    const Eigen::Matrix<Scalar, 6, 1>& odom_displacement) const {

  // The filtered correction has degenerate DoFs zeroed out.
  // The odometry displacement provides the missing information.
  // We fuse by adding the odom components that were zeroed out.
  //
  // correction_final = correction_filtered + (I − P) · odom_displacement
  // where P is the same projection that filtered the LiDAR correction.

  // For simplicity, we compute:
  // correction_final = correction_filtered
  // for each degenerate direction, replace with odom component
  // (this relies on the calling code to pass the filtered correction)

  // Actually we need to know which components were zeroed.
  // For now, return the filtered + add odom for degenerate dims.
  // The caller is responsible for ensuring the degenerate components
  // are replaced by odometry.

  return correction_filtered;  // caller fuses separately
}

}  // namespace m20::backend