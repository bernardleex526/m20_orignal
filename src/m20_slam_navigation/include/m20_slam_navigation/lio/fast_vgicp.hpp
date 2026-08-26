#pragma once
/**
 * @file fast_vgicp.hpp
 * @brief Multi-threaded Voxel-Accelerated Generalized ICP (FastVAGICP).
 *
 * VGICP extends GICP by using voxel distributions instead of per-point covariances
 * to achieve O(N) complexity vs. O(N²) for standard GICP.
 *
 * Cost function (distribution-to-distribution):
 *   For each correspondence (source point p_i, target voxel (μ_j, Σ_j)):
 *     e_i = (R·p_i + t − μ_j)
 *     C_i = R·Σ_{p_i}·Rᵀ + Σ_j    (assuming isotropic Σ_{p_i} = σ²·I)
 *     r_i = e_iᵀ · C_i⁻¹ · e_i
 *
 *   Total cost:  E(R, t) = Σ_i r_i
 *
 * The optimization is performed via Gauss-Newton on the SE(3) manifold:
 *   δξ = −(JᵀWJ)⁻¹ JᵀW e
 *   T ← T ∘ exp(δξ)
 *
 * Multi-threading: correspondences are split across OpenMP threads.
 * Target: < 15ms per registration on Intel x86 / Jetson Orin.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/lio/voxel_map.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace m20::lio {

struct VGICPResult {
  SE3Pose                          T_world_lidar;       ///< optimized transform
  Eigen::Matrix<Scalar, 6, 6>      information;         ///< Hessian (JᵀWJ) at convergence
  double                           fitness;             ///< final cost / N
  int                              iterations;          ///< iterations taken
  bool                             converged;
  double                           elapsed_ms;          ///< wall-clock time
};

class FastVAGICP {
public:
  explicit FastVAGICP(const LIOParams& params);

  /**
   * @brief Align source point cloud to target voxel map.
   *
   * @param source         Input point cloud (LiDAR frame)
   * @param voxel_map      Target voxel map (world frame)
   * @param T_init         Initial guess for T_world_lidar
   * @return               Registration result
   */
  VGICPResult align(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
                    const std::shared_ptr<VoxelMap>& voxel_map,
                    const SE3Pose& T_init);

  /// Set registration parameters
  void setParams(const LIOParams& params) { params_ = params; }

private:
  /// Compute Jacobian J_i (6×6) and residual e_i (3×1) for one correspondence
  void computeJacobianAndResidual(
      const Eigen::Matrix<Scalar, 3, 1>& source_point,
      const VoxelEntry& target_voxel,
      const SE3Pose& T,
      Eigen::Matrix<Scalar, 3, 6>& J_i,
      Eigen::Matrix<Scalar, 3, 1>& e_i,
      Eigen::Matrix<Scalar, 3, 3>& weight) const;

  LIOParams params_;
  SE3Pose T_lidar_imu_;  ///< LiDAR-in-IMU extrinsic (identity if already in LiDAR frame)
};

}  // namespace m20::lio