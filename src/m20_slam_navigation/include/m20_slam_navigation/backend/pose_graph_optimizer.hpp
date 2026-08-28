#pragma once
/**
 * @file pose_graph_optimizer.hpp
 * @brief Back-end pose graph optimization orchestrator.
 *
 * Coordinates the factor graph, loop closure detection, and degeneracy
 * handling. Runs asynchronously from the LIO front-end:
 *   - Receives odometry factors from LIO
 *   - Receives keyframe point clouds for loop detection
 *   - Runs iSAM2 optimization
 *   - Publishes optimized trajectory
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/backend/factor_graph.hpp"
#include "m20_slam_navigation/backend/degeneracy_detector.hpp"
#include "m20_slam_navigation/backend/loop_closure.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace m20::backend {

using OptimizedPoseCallback = std::function<void(FrameId frame_id, const SE3Pose& pose)>;
using LoopClosureCallback = std::function<void(const LoopCandidate& candidate)>;

class PoseGraphOptimizer {
public:
  PoseGraphOptimizer(const BackendParams& params);

  /// Add odometry factor (called from LIO)
  void addOdometry(FrameId prev_id, FrameId curr_id,
                   const SE3Pose& relative_pose,
                   const Eigen::Matrix<Scalar, 6, 6>& information);

  /// Add keyframe for loop closure detection (called from LIO)
  void addKeyframe(FrameId frame_id, const SE3Pose& pose,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  /// Add prior constraint (from relocalization)
  void addPriorPose(FrameId frame_id, const SE3Pose& pose,
                    const Eigen::Matrix<Scalar, 6, 6>& covariance);

  /// Add the IMU gravity-direction factor for a keyframe.
  void addGravityFactor(FrameId frame_id,
                        const Eigen::Matrix<Scalar, 3, 1>& gravity_body);

  /// Queue a geometrically verified loop candidate for the next optimize().
  void addLoopClosure(const LoopCandidate& candidate);

  /// Run optimization (called periodically or on new data)
  void optimize();

  /// Get optimized pose
  SE3Pose getPose(FrameId frame_id) const;

  /// Get full trajectory
  std::vector<std::pair<FrameId, SE3Pose>> getTrajectory() const;

  /// Callbacks
  void setOptimizedPoseCallback(OptimizedPoseCallback cb) { optimized_cb_ = std::move(cb); }
  void setLoopClosureCallback(LoopClosureCallback cb) { loop_cb_ = std::move(cb); }

  /// Reset
  void reset();

private:
  BackendParams                         params_;
  std::unique_ptr<FactorGraph>          factor_graph_;
  std::unique_ptr<DegeneracyDetector>   degeneracy_detector_;
  std::unique_ptr<LoopClosureDetector>  loop_detector_;

  OptimizedPoseCallback                 optimized_cb_;
  LoopClosureCallback                   loop_cb_;

  mutable std::mutex                    mutex_;
  std::vector<LoopCandidate>            pending_loops_;
  std::size_t                            keyframe_counter_{0};
};

}  // namespace m20::backend
