#include "m20_slam_navigation/backend/pose_graph_optimizer.hpp"

#include <algorithm>
#include <thread>

namespace m20::backend {

PoseGraphOptimizer::PoseGraphOptimizer(const BackendParams& params)
    : params_(params)
    , factor_graph_(std::make_unique<FactorGraph>(params))
    , degeneracy_detector_(std::make_unique<DegeneracyDetector>(params))
    , loop_detector_(std::make_unique<LoopClosureDetector>(params)) {
}

void PoseGraphOptimizer::addOdometry(
    FrameId prev_id, FrameId curr_id,
    const SE3Pose& relative_pose,
    const Eigen::Matrix<Scalar, 6, 6>& information) {

  std::lock_guard<std::mutex> lock(mutex_);

  // Degeneracy check on the information matrix before adding
  Eigen::Matrix<Scalar, 3, 1> heading =
      relative_pose.q._transformVector(Eigen::Matrix<Scalar, 3, 1>(1, 0, 0));

  DegeneracyResult deg = degeneracy_detector_->analyze(information, heading);

  if (deg.is_degenerate && params_.enable_degeneracy_fallback) {
    // Use filtered information (degenerate dimensions zeroed) to avoid
    // corrupting the factor graph with bad constraints
    factor_graph_->addOdometryFactor(prev_id, curr_id, relative_pose, deg.filtered_hessian);
  } else {
    factor_graph_->addOdometryFactor(prev_id, curr_id, relative_pose, information);
  }
}

void PoseGraphOptimizer::addKeyframe(
    FrameId frame_id, const SE3Pose& pose,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

  std::lock_guard<std::mutex> lock(mutex_);

  if (!params_.enable_loop_closure) {
    return;
  }

  // Build ScanContext descriptor and add to database
  auto desc = LoopClosureDetector::buildDescriptor(cloud, frame_id, pose);
  loop_detector_->addKeyframe(desc);

  // Geometric verification is intentionally low-frequency. Descriptor
  // insertion remains per-keyframe, while ICP/GHT runs every Nth keyframe so
  // the LIO thread can keep up with the complete offline bag.
  ++keyframe_counter_;
  if (params_.loop_detection_stride > 1 &&
      (keyframe_counter_ % static_cast<std::size_t>(params_.loop_detection_stride)) != 0U) {
    return;
  }

  // Check for loop closures
  auto loops = loop_detector_->detectLoop(desc);
  for (const auto& loop : loops) {
    pending_loops_.push_back(loop);
    if (loop_cb_) {
      loop_cb_(loop);
    }
  }
}

void PoseGraphOptimizer::addPriorPose(
    FrameId frame_id, const SE3Pose& pose,
    const Eigen::Matrix<Scalar, 6, 6>& covariance) {

  std::lock_guard<std::mutex> lock(mutex_);
  factor_graph_->addPriorFactor(frame_id, pose, covariance);
}

void PoseGraphOptimizer::addGravityFactor(
    FrameId frame_id, const Eigen::Matrix<Scalar, 3, 1>& gravity_body) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (params_.enable_imu_gravity) {
    factor_graph_->addGravityFactor(frame_id, gravity_body);
  }
}

void PoseGraphOptimizer::addLoopClosure(const LoopCandidate& candidate) {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_loops_.push_back(candidate);
  if (loop_cb_) {
    loop_cb_(candidate);
  }
}

void PoseGraphOptimizer::optimize() {
  std::lock_guard<std::mutex> lock(mutex_);

  // Insert pending loop closure factors
  for (const auto& loop : pending_loops_) {
    factor_graph_->addLoopClosureFactor(loop.src_frame, loop.tgt_frame, loop.relative_pose);
  }
  pending_loops_.clear();

  // Run iSAM2
  factor_graph_->optimize();

  // Notify optimized poses
  if (optimized_cb_) {
    auto poses = factor_graph_->getAllPoses();
    for (const auto& [id, pose] : poses) {
      optimized_cb_(id, pose);
    }
  }
}

SE3Pose PoseGraphOptimizer::getPose(FrameId frame_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return factor_graph_->getPoseEstimate(frame_id);
}

std::vector<std::pair<FrameId, SE3Pose>> PoseGraphOptimizer::getTrajectory() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return factor_graph_->getAllPoses();
}

void PoseGraphOptimizer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  factor_graph_->reset();
  loop_detector_->clear();
  pending_loops_.clear();
  keyframe_counter_ = 0;
}

}  // namespace m20::backend
