#pragma once
/**
 * @file ndt_matcher.hpp
 * @brief 3D Normal Distributions Transform (NDT) scan-to-map registration.
 *
 * NDT models the target point cloud as a collection of Gaussian distributions
 * over a regular voxel grid. Registration maximizes the likelihood of source
 * points under these distributions.
 *
 * Cost (negative log-likelihood):
 *   E(T) = −Σ_i log( exp(−½ (T·p_i − μ_j)ᵀ Σ_j⁻¹ (T·p_i − μ_j)) )
 *
 * The optimization uses Newton's method with a robust Huber kernel to handle
 * outliers (dynamic objects, sensor noise).
 *
 * This is used for:
 *  1. Global relocalization (multi-hypothesis matching).
 *  2. Real-time pose tracking against the prior map.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace m20::localization {

struct NDTResult {
  SE3Pose                          T_world_lidar;
  Eigen::Matrix<Scalar, 6, 6>      information{
    Eigen::Matrix<Scalar, 6, 6>::Zero()};
  double                           fitness_score{1e9};
  double                           transformation_probability{0.0};
  int                              iterations{0};
  bool                             converged{false};
};

class NDTMatcher {
public:
  explicit NDTMatcher(const LocalizationParams& params);

  /**
   * @brief Set the target (prior) map. Must be called before align().
   */
  void setTargetMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map);

  /**
   * @brief Align source cloud to target map via 3D NDT.
   *
   * @param source   Input scan (LiDAR frame)
   * @param T_init   Initial pose guess
   * @return         Registration result
   */
  NDTResult align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                  const SE3Pose& T_init);

  /**
   * @brief Multi-hypothesis global relocalization.
   *
   * Generates multiple initial pose hypotheses within a search region
   * and returns the best NDT match.
   *
   * @param source       Input scan
   * @param search_center Center of search region
   * @param search_radius_xy Horizontal search radius [m]
   * @param num_hypotheses Number of hypotheses to test
   * @return Best NDTResult (fitness_score = 1.0 if no match found)
   */
  NDTResult globalRelocalize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                             const Eigen::Matrix<Scalar, 3, 1>& search_center,
                             Scalar search_radius_xy,
                             int num_hypotheses);

  /// Get the NDT voxel grid (for visualization)
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& getTargetMap() const { return target_map_; }

private:
  LocalizationParams params_;

  // PCL NDT implementation is used internally; we wrap for our types
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_map_;
};

}  // namespace m20::localization
