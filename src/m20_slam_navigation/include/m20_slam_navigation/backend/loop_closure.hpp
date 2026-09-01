#pragma once
/**
 * @file loop_closure.hpp
 * @brief Loop closure detection via Scan Context descriptor matching.
 *
 * Scan Context (Kim & Kim, 2018) encodes a LiDAR scan as a 2D polar image:
 * rows = rings (elevation bins), columns = azimuth sectors. The descriptor
 * is rotation-invariant via column-wise shifting.
 *
 * For loop detection:
 *   1. Build ScanContext descriptor for each keyframe.
 *   2. Query nearest neighbours in descriptor space (ring-key for fast pre-filter).
 *   3. Geometric verification via ICP on the candidate pair.
 *   4. If fitness < threshold, add loop closure factor to back-end.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <deque>
#include <memory>
#include <vector>

namespace m20::backend {

/// Scan Context descriptor: Nr rings × Ns sectors matrix
struct ScanContextDescriptor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNumRings   = 20;
  static constexpr int kNumSectors = 60;

  Eigen::Matrix<Scalar, kNumRings, kNumSectors> data{
    Eigen::Matrix<Scalar, kNumRings, kNumSectors>::Zero()};
  Eigen::Matrix<Scalar, 1, kNumRings> ring_key{
    Eigen::Matrix<Scalar, 1, kNumRings>::Zero()};  ///< for fast candidate search
  FrameId frame_id{INVALID_FRAME_ID};
  SE3Pose pose;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud{
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()};

  /// Compute cosine distance to another descriptor, searching best column alignment
  Scalar distance(const ScanContextDescriptor& other) const;
};

class LoopClosureDetector {
public:
  explicit LoopClosureDetector(const BackendParams& params);

  /**
   * @brief Build Scan Context descriptor from a deskewed point cloud.
   */
  static ScanContextDescriptor buildDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
      FrameId frame_id, const SE3Pose& pose,
      Scalar max_range = 80.0);

  /**
   * @brief Add a keyframe descriptor to the database.
   */
  void addKeyframe(const ScanContextDescriptor& desc);

  /**
   * @brief Detect loop closures for the latest keyframe.
   *
   * @return List of detected loop candidates (src = newly added, tgt = historical)
   */
  std::vector<LoopCandidate> detectLoop(const ScanContextDescriptor& query_desc);

  /// Get database size
  std::size_t databaseSize() const { return database_.size(); }

  /// Clear database
  void clear();

private:
  BackendParams                               params_;
  std::deque<ScanContextDescriptor>           database_;
  static constexpr std::size_t kMaxDatabase  = 1000;
};

}  // namespace m20::backend
