#include "m20_slam_navigation/lio/fast_vgicp.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <limits>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace m20::lio {

FastVAGICP::FastVAGICP(const LIOParams& params) : params_(params) {}

VGICPResult FastVAGICP::align(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const std::shared_ptr<VoxelMap>& voxel_map,
    const SE3Pose& T_init) {

  auto t_start = std::chrono::steady_clock::now();

  VGICPResult result;
  result.T_world_lidar = T_init;
  result.fitness = 1e9;
  result.converged = false;

  SE3Pose T = T_init;
  Scalar prev_cost = 1e9;
  Scalar lambda = 1e-3;  // LM damping factor (initial)
  Eigen::Matrix<Scalar, 6, 6> final_hessian =
    Eigen::Matrix<Scalar, 6, 6>::Identity() * Scalar(1e-6);
  int completed_iterations = 0;
  Scalar last_update_norm = std::numeric_limits<Scalar>::infinity();

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    completed_iterations = iter + 1;
    // Transform source to world frame using current T
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_transformed(
        new pcl::PointCloud<pcl::PointXYZI>());
    source_transformed->reserve(source->size());
    for (const auto& pt : source->points) {
      Eigen::Matrix<Scalar, 3, 1> p(pt.x, pt.y, pt.z);
      Eigen::Matrix<Scalar, 3, 1> pw = T.transformPoint(p);
      pcl::PointXYZI pw_pt;
      pw_pt.x = pw.x(); pw_pt.y = pw.y(); pw_pt.z = pw.z();
      pw_pt.intensity = pt.intensity;
      source_transformed->push_back(pw_pt);
    }

    // Find correspondences
    auto correspondences = voxel_map->findCorrespondences(
        source_transformed, params_.correspondence_radius);
    result.correspondences = static_cast<int>(correspondences.size());

    if (correspondences.size() < 10) {
      result.converged = false;
      break;
    }

    // Build Gauss-Newton system: H·δξ = −b
    // H = Σ J_iᵀ W_i J_i,  b = Σ J_iᵀ W_i e_i
    Eigen::Matrix<Scalar, 6, 6> H = Eigen::Matrix<Scalar, 6, 6>::Zero();
    Eigen::Matrix<Scalar, 6, 1> b = Eigen::Matrix<Scalar, 6, 1>::Zero();
    Scalar total_cost = 0;

    // Process correspondences (with OpenMP for speed)
    int n_corr = static_cast<int>(correspondences.size());

    // Per-thread accumulators for thread safety
#ifdef _OPENMP
    int n_threads = omp_get_max_threads();
    std::vector<Eigen::Matrix<Scalar, 6, 6>> H_threads(n_threads, Eigen::Matrix<Scalar, 6, 6>::Zero());
    std::vector<Eigen::Matrix<Scalar, 6, 1>> b_threads(n_threads, Eigen::Matrix<Scalar, 6, 1>::Zero());
    std::vector<Scalar> cost_threads(n_threads, 0);

#pragma omp parallel for num_threads(params_.num_threads)
    for (int i = 0; i < n_corr; ++i) {
#else
    Eigen::Matrix<Scalar, 6, 6> H_acc = Eigen::Matrix<Scalar, 6, 6>::Zero();
    Eigen::Matrix<Scalar, 6, 1> b_acc = Eigen::Matrix<Scalar, 6, 1>::Zero();
    Scalar cost_acc = 0;
    for (int i = 0; i < n_corr; ++i) {
#endif
      const auto& [src_idx, voxel_ptr] = correspondences[i];
      const auto& pt = source->points[src_idx];
      Eigen::Matrix<Scalar, 3, 1> p(pt.x, pt.y, pt.z);

      // Compute Jacobian (3×6), residual (3×1), and weight matrix (3×3)
      Eigen::Matrix<Scalar, 3, 6> J;
      Eigen::Matrix<Scalar, 3, 1> e;
      Eigen::Matrix<Scalar, 3, 3> W;
      computeJacobianAndResidual(p, *voxel_ptr, T, J, e, W);

      // Accumulate
      Eigen::Matrix<Scalar, 6, 6> JtWJ = J.transpose() * W * J;
      Eigen::Matrix<Scalar, 6, 1> JtWe = J.transpose() * W * e;
      Scalar cost = e.transpose() * W * e;

#ifdef _OPENMP
      int tid = omp_get_thread_num();
      H_threads[tid] += JtWJ;
      b_threads[tid] += JtWe;
      cost_threads[tid] += cost;
    }
    // Reduce threads
    for (int t = 0; t < n_threads; ++t) {
      H += H_threads[t];
      b += b_threads[t];
      total_cost += cost_threads[t];
      H_threads[t].setZero();
      b_threads[t].setZero();
      cost_threads[t] = 0;
    }
#else
      H_acc += JtWJ;
      b_acc += JtWe;
      cost_acc += cost;
    }
    H = H_acc;
    b = b_acc;
    total_cost = cost_acc;
#endif

    total_cost /= static_cast<Scalar>(correspondences.size());
    result.fitness = static_cast<double>(total_cost);
    final_hessian = H;

    // Check convergence
    const Scalar signed_cost_change = total_cost - prev_cost;
    const Scalar relative_cost_change =
      std::abs(signed_cost_change) / (std::abs(prev_cost) + 1e-6);
    if (relative_cost_change < params_.convergence_threshold && iter > 3) {
      result.converged = true;
      break;
    }
    prev_cost = total_cost;

    // Damped Gauss-Newton: (H + λI)·δξ = −b
    Eigen::Matrix<Scalar, 6, 6> H_damped = H + lambda * Eigen::Matrix<Scalar, 6, 6>::Identity();

    Eigen::Matrix<Scalar, 6, 1> delta_xi;
    // Use LDLT for symmetric positive-definite (after damping)
    Eigen::LDLT<Eigen::Matrix<Scalar, 6, 6>> solver(H_damped);

    if (solver.info() == Eigen::Success) {
      delta_xi = solver.solve(-b);
      last_update_norm = delta_xi.norm();

      // The Jacobian below is a left perturbation Jacobian.
      SE3Pose dT = SE3Pose::exp(delta_xi);
      T = dT * T;

      // Adjust damping (Levenberg-Marquardt style)
      if (signed_cost_change < 0) {
        lambda *= 0.5;  // cost decreased: reduce damping
      } else {
        lambda *= 2.0;  // cost increased: increase damping
      }
      lambda = math::clamp(lambda, Scalar(1e-6), Scalar(1e6));
    } else {
      // Singular system: increase damping
      lambda *= 10.0;
      if (lambda > 1e6) break;
    }
  }

  if (!result.converged && completed_iterations >= 5 && std::isfinite(result.fitness) &&
      std::isfinite(last_update_norm) && last_update_norm < Scalar(0.05)) {
    result.converged = true;
  }
  result.iterations = completed_iterations;
  result.final_update_norm = static_cast<double>(last_update_norm);
  result.T_world_lidar = T;
  // Information matrix = H at convergence
  result.information = final_hessian;

  auto t_end = std::chrono::steady_clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  return result;
}

void FastVAGICP::computeJacobianAndResidual(
    const Eigen::Matrix<Scalar, 3, 1>& source_point,
    const VoxelEntry& target_voxel,
    const SE3Pose& T,
    Eigen::Matrix<Scalar, 3, 6>& J_i,
    Eigen::Matrix<Scalar, 3, 1>& e_i,
    Eigen::Matrix<Scalar, 3, 3>& weight) const {

  // Residual: e = (R·p + t) − μ
  Eigen::Matrix<Scalar, 3, 1> p_transformed = T.transformPoint(source_point);
  e_i = p_transformed - target_voxel.centroid;

  // Jacobian of transformed point w.r.t. se(3) twist:
  // ∂(R·p + t)/∂ξ = [ −[R·p]×  |  I₃ ]
  Eigen::Matrix<Scalar, 3, 3> R = T.q.toRotationMatrix();
  (void)R;

  J_i.template block<3, 3>(0, 0) = -math::skew(p_transformed);
  J_i.template block<3, 3>(0, 3) = Eigen::Matrix<Scalar, 3, 3>::Identity();  // I₃

  // Weight matrix: C = R·σ²·I·Rᵀ + Σ_target
  // Using isotropic source covariance σ² = voxel_size² / 4 (typical GICP)
  Scalar sigma2 = params_.voxel_size * params_.voxel_size / 4.0;
  Eigen::Matrix<Scalar, 3, 3> C = sigma2 * Eigen::Matrix<Scalar, 3, 3>::Identity();

  // Add target voxel covariance (normalized by point count)
  if (target_voxel.point_count > 1) {
    Scalar n = static_cast<Scalar>(target_voxel.point_count);
    C += target_voxel.covariance / (n - 1.0);
  }

  // Regularize for numerical stability
  C.diagonal() += Eigen::Matrix<Scalar, 3, 1>::Constant(1e-6);

  // Weight = C⁻¹
  weight = C.inverse();
}

}  // namespace m20::lio
