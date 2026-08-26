#pragma once
/**
 * @file degeneracy_detector.hpp
 * @brief Detects geometric degeneracy and performs directional decoupling.
 *
 * In long corridors, tunnels, or featureless hallways typical of indoor
 * quadruped operations, the LiDAR cannot observe longitudinal translation
 * (forward direction). The Hessian H = JᵀWJ becomes rank-deficient along
 * the corridor direction.
 *
 * Algorithm (Zhang et al. 2016, "On Degeneracy of Optimization-based State
 * Estimation Problems"):
 *   1. Compute eigenvalue decomposition: H = V·Λ·Vᵀ.
 *   2. Identify eigenvalues λ_i < Threshold_deg.
 *   3. For each degenerate eigenvalue, check if its eigenvector v_i aligns
 *      with the robot's longitudinal (X) axis (|v_i · [1,0,0]| > 0.9).
 *   4. If degenerate in the heading direction:
 *        a. Zero out the LiDAR correction component along v_deg.
 *        b. Use foot odometry / IMU pre-integration for longitudinal displacement.
 *        c. Keep LiDAR constraints for lateral (Y), vertical (Z), and yaw.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Dense>

#include <vector>

namespace m20::backend {

struct DegeneracyResult {
  bool                               is_degenerate{false};
  Eigen::Matrix<Scalar, 6, 1>        degenerate_directions;  ///< binary mask of degenerate DoFs
  std::vector<Eigen::Matrix<Scalar, 6, 1>> degenerate_eigenvectors;
  std::vector<Scalar>                      eigenvalues;
  Eigen::Matrix<Scalar, 6, 6>             filtered_hessian;  ///< Hessian with degenerate dims zeroed
};

class DegeneracyDetector {
public:
  explicit DegeneracyDetector(const BackendParams& params);

  /**
   * @brief Analyze Hessian matrix for degeneracy.
   *
   * @param H                    6×6 Hessian (JᵀWJ) from VGICP registration
   * @param robot_heading        Unit vector of robot's forward direction in world frame
   * @return                     Degeneracy analysis result
   */
  DegeneracyResult analyze(const Eigen::Matrix<Scalar, 6, 6>& H,
                           const Eigen::Matrix<Scalar, 3, 1>& robot_heading);

  /**
   * @brief Filter LiDAR correction vector to remove degenerate components.
   *
   * δξ_filtered = (I − Σ v_i·v_iᵀ) · δξ_raw
   * where v_i are the degenerate eigenvectors that align with robot heading.
   *
   * @param correction_raw       6×1 raw correction from Gauss-Newton
   * @param degeneracy           Degeneracy analysis result
   * @return                     Filtered correction (degenerate DoFs zeroed)
   */
  Eigen::Matrix<Scalar, 6, 1> filterCorrection(
      const Eigen::Matrix<Scalar, 6, 1>& correction_raw,
      const DegeneracyResult& degeneracy) const;

  /// Replace degenerate displacement with odometry estimate
  Eigen::Matrix<Scalar, 6, 1> fuseOdometryCorrection(
      const Eigen::Matrix<Scalar, 6, 1>& correction_filtered,
      const Eigen::Matrix<Scalar, 6, 1>& odom_displacement) const;

private:
  BackendParams params_;
};

}  // namespace m20::backend