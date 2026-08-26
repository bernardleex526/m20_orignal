#include "m20_slam_navigation/backend/loop_closure.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace m20::backend {

// =============================================================================
// ScanContextDescriptor
// =============================================================================

Scalar ScanContextDescriptor::distance(const ScanContextDescriptor& other) const {
  // Cosine distance with column-wise shift for rotation invariance
  Scalar min_dist = std::numeric_limits<Scalar>::max();

  for (int shift = 0; shift < kNumSectors; ++shift) {
    Scalar dist = 0;
    for (int r = 0; r < kNumRings; ++r) {
      for (int s = 0; s < kNumSectors; ++s) {
        int s_shifted = (s + shift) % kNumSectors;
        dist += data(r, s) * other.data(r, s_shifted);
      }
    }
    // Cosine distance = 1 − (A·B) / (‖A‖·‖B‖)
    Scalar norm = data.norm() * other.data.norm();
    if (norm > 1e-10) {
      dist = 1.0 - dist / norm;
      if (dist < min_dist) min_dist = dist;
    }
  }

  return min_dist;
}

// =============================================================================
// LoopClosureDetector
// =============================================================================

LoopClosureDetector::LoopClosureDetector(const BackendParams& params)
    : params_(params) {}

ScanContextDescriptor LoopClosureDetector::buildDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    FrameId frame_id, const SE3Pose& pose,
    Scalar max_range) {

  ScanContextDescriptor desc;
  desc.frame_id = frame_id;
  desc.pose = pose;
  desc.data.setZero();
  desc.ring_key.setZero();

  if (cloud->empty()) return desc;

  // Parameters
  constexpr Scalar kMinRange = 0.5;
  constexpr int kNumRings   = ScanContextDescriptor::kNumRings;
  constexpr int kNumSectors = ScanContextDescriptor::kNumSectors;
  constexpr Scalar kMaxRange = 80.0;

  Scalar ring_step = (kMaxRange - kMinRange) / static_cast<Scalar>(kNumRings);
  Scalar sector_step = 2.0 * math::kPI / static_cast<Scalar>(kNumSectors);

  // Accumulate max height per bin
  Eigen::Matrix<Scalar, kNumRings, kNumSectors> max_height;
  max_height.setConstant(-1e9);

  for (const auto& pt : cloud->points) {
    Scalar r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    if (r < kMinRange || r > kMaxRange) continue;

    Scalar theta = std::atan2(pt.y, pt.x);
    if (theta < 0) theta += 2.0 * math::kPI;

    int ring_idx = static_cast<int>((r - kMinRange) / ring_step);
    ring_idx = std::clamp(ring_idx, 0, kNumRings - 1);

    int sector_idx = static_cast<int>(theta / sector_step);
    sector_idx = std::clamp(sector_idx, 0, kNumSectors - 1);

    if (pt.z > max_height(ring_idx, sector_idx)) {
      max_height(ring_idx, sector_idx) = pt.z;
    }
  }

  // Copy to descriptor, replacing -inf with 0
  for (int r = 0; r < kNumRings; ++r) {
    for (int s = 0; s < kNumSectors; ++s) {
      desc.data(r, s) = (max_height(r, s) > -1e8) ? max_height(r, s) : 0;
    }
  }

  // Compute ring key: mean height per ring (for fast candidate search)
  for (int r = 0; r < kNumRings; ++r) {
    Scalar sum = 0;
    int count = 0;
    for (int s = 0; s < kNumSectors; ++s) {
      if (max_height(r, s) > -1e8) {
        sum += max_height(r, s);
        count++;
      }
    }
    desc.ring_key(r) = (count > 0) ? (sum / count) : 0;
  }

  return desc;
}

void LoopClosureDetector::addKeyframe(const ScanContextDescriptor& desc) {
  database_.push_back(desc);
  while (database_.size() > kMaxDatabase) {
    database_.pop_front();
  }
}

std::vector<LoopCandidate> LoopClosureDetector::detectLoop(
    const ScanContextDescriptor& query_desc) {

  std::vector<LoopCandidate> candidates;

  if (database_.size() < 2) return candidates;

  // Step 1: Ring-key pre-filter (fast cosine distance)
  struct Candidate {
    std::size_t idx;
    Scalar ring_key_dist;
    Scalar scan_context_dist;
  };

  std::vector<Candidate> ring_candidates;
  Scalar query_rk_norm = query_desc.ring_key.norm();
  if (query_rk_norm < 1e-10) return candidates;

  for (std::size_t i = 0; i < database_.size() - 1; ++i) {
    const auto& db_desc = database_[i];

    // Skip nearby frames (same sequence, < 50 frames apart)
    if (std::abs(static_cast<int64_t>(query_desc.frame_id) -
                 static_cast<int64_t>(db_desc.frame_id)) < 50) {
      continue;
    }

    // Ring key cosine distance
    Scalar rk_norm = db_desc.ring_key.norm();
    if (rk_norm < 1e-10) continue;

    Scalar cos_dist = 1.0 - query_desc.ring_key.dot(db_desc.ring_key) / (query_rk_norm * rk_norm);
    if (cos_dist < kRingKeyThreshold) {
      ring_candidates.push_back({i, cos_dist, 0});
    }
  }

  if (ring_candidates.empty()) return candidates;

  // Step 2: ScanContext full distance for top candidates
  for (auto& cand : ring_candidates) {
    cand.scan_context_dist = query_desc.distance(database_[cand.idx]);
  }

  // Sort by scan context distance (ascending)
  std::sort(ring_candidates.begin(), ring_candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              return a.scan_context_dist < b.scan_context_dist;
            });

  // Take top kNumCandidates
  for (int i = 0; i < std::min(kNumCandidates, static_cast<int>(ring_candidates.size())); ++i) {
    const auto& cand = ring_candidates[i];
    if (cand.scan_context_dist < kScanContextThreshold) {
      LoopCandidate loop;
      loop.src_frame = query_desc.frame_id;
      loop.tgt_frame = database_[cand.idx].frame_id;
      // Initial estimate: identity (need ICP for refinement)
      loop.relative_pose = database_[cand.idx].pose.inverse() * query_desc.pose;
      loop.fitness_score = static_cast<Scalar>(cand.scan_context_dist);
      candidates.push_back(loop);
    }
  }

  return candidates;
}

void LoopClosureDetector::clear() {
  database_.clear();
}

}  // namespace m20::backend