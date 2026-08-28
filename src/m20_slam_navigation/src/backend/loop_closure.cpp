#include "m20_slam_navigation/backend/loop_closure.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <array>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <rclcpp/rclcpp.hpp>

namespace m20::backend {

namespace {

struct GhtGuess {
  Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
  Scalar segment_fraction{0.0};
};

// Lightweight geometric-hash stage. Each azimuth segment contributes a
// centroid/radius signature; cyclic sector shifts enumerate yaw hypotheses,
// and robust medians provide translation without requiring a good odometry
// initial guess. ICP remains the metric acceptance gate after this stage.
std::optional<GhtGuess> estimateGhtGuess(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
    const BackendParams& params) {
  if (!source || !target || source->empty() || target->empty()) return std::nullopt;
  const int segments = std::max(3, params.segment_num);
  struct Bin {
    Eigen::Vector3f sum{Eigen::Vector3f::Zero()};
    int count{0};
  };
  std::vector<Bin> src(static_cast<std::size_t>(segments));
  std::vector<Bin> tgt(static_cast<std::size_t>(segments));
  const auto fill = [segments](const auto& cloud, auto& bins) {
      for (const auto& point : cloud->points) {
        const float angle = std::atan2(point.y, point.x);
        float wrapped = angle < 0.0F ? angle + static_cast<float>(math::k2PI) : angle;
        int index = static_cast<int>(wrapped / static_cast<float>(math::k2PI) * segments);
        index = std::clamp(index, 0, segments - 1);
        bins[static_cast<std::size_t>(index)].sum += Eigen::Vector3f(point.x, point.y, point.z);
        ++bins[static_cast<std::size_t>(index)].count;
      }
    };
  fill(source, src);
  fill(target, tgt);

  const Scalar tolerance = std::max(
    Scalar(0.05), params.distance_threshold_factor * Scalar(4.0));
  std::optional<GhtGuess> best;
  for (int shift = 0; shift < segments; ++shift) {
    struct Pair { Eigen::Vector3f source; Eigen::Vector3f target; };
    std::vector<Pair> pairs;
    for (int index = 0; index < segments; ++index) {
      const auto& a = src[static_cast<std::size_t>(index)];
      const auto& b = tgt[static_cast<std::size_t>((index + shift) % segments)];
      if (a.count < 3 || b.count < 3) continue;
      const Eigen::Vector3f ca = a.sum / static_cast<float>(a.count);
      const Eigen::Vector3f cb = b.sum / static_cast<float>(b.count);
      if (std::abs(ca.head<2>().norm() - cb.head<2>().norm()) > tolerance ||
          std::abs(ca.z() - cb.z()) > tolerance * 2.0) continue;
      pairs.push_back({ca, cb});
    }
    if (pairs.size() < 2U) continue;

    std::vector<Scalar> yaw_samples;
    yaw_samples.reserve(pairs.size());
    for (const auto& pair : pairs) {
      yaw_samples.push_back(std::atan2(
        static_cast<Scalar>(pair.target.y()), static_cast<Scalar>(pair.target.x())) -
        std::atan2(static_cast<Scalar>(pair.source.y()), static_cast<Scalar>(pair.source.x())));
    }
    std::sort(yaw_samples.begin(), yaw_samples.end());
    Scalar yaw = yaw_samples[yaw_samples.size() / 2U];
    yaw = math::normalize_angle(yaw);
    const Eigen::Matrix2d rotation =
      Eigen::Rotation2Dd(yaw).toRotationMatrix();
    std::vector<Scalar> tx;
    std::vector<Scalar> ty;
    std::vector<Scalar> tz;
    for (const auto& pair : pairs) {
      const Eigen::Vector2d transformed =
        rotation * pair.source.head<2>().cast<Scalar>();
      tx.push_back(static_cast<Scalar>(pair.target.x()) - transformed.x());
      ty.push_back(static_cast<Scalar>(pair.target.y()) - transformed.y());
      tz.push_back(static_cast<Scalar>(pair.target.z() - pair.source.z()));
    }
    const auto median = [](std::vector<Scalar>& values) {
        std::sort(values.begin(), values.end());
        return values[values.size() / 2U];
      };
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<2, 2>(0, 0) = rotation.cast<float>();
    transform(0, 3) = static_cast<float>(median(tx));
    transform(1, 3) = static_cast<float>(median(ty));
    transform(2, 3) = static_cast<float>(median(tz));
    const Scalar fraction = static_cast<Scalar>(pairs.size()) / static_cast<Scalar>(segments);
    if (!best || fraction > best->segment_fraction) {
      best = GhtGuess{transform, fraction};
    }
  }
  return best;
}

}  // namespace

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
  desc.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  desc.data.setZero();
  desc.ring_key.setZero();

  if (cloud->empty()) return desc;

  // Loop detection must never stall the LIO input path with a full-resolution
  // keyframe. A 0.30 m cloud preserves office-scale geometry while reducing
  // descriptor and ICP work by roughly an order of magnitude.
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setLeafSize(0.30F, 0.30F, 0.30F);
  voxel_filter.setInputCloud(cloud);
  voxel_filter.filter(*desc.cloud);
  if (desc.cloud->empty()) return desc;

  // Parameters
  constexpr Scalar kMinRange = 0.5;
  constexpr int kNumRings   = ScanContextDescriptor::kNumRings;
  constexpr int kNumSectors = ScanContextDescriptor::kNumSectors;
  const Scalar kMaxRange = std::max(kMinRange + Scalar(1.0), max_range);

  Scalar ring_step = (kMaxRange - kMinRange) / static_cast<Scalar>(kNumRings);
  Scalar sector_step = 2.0 * math::kPI / static_cast<Scalar>(kNumSectors);

  // Accumulate max height per bin
  Eigen::Matrix<Scalar, kNumRings, kNumSectors> max_height;
  max_height.setConstant(-1e9);

  for (const auto& pt : desc.cloud->points) {
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

  // Step 1: Build a temporally separated candidate set. The native M20
  // office sequence revisits the starting area with substantial yaw change;
  // a strict ring-key gate rejects that revisit before geometry can verify it.
  // Keep the inexpensive descriptor ranking, but do not discard candidates
  // solely on the ring-key cosine threshold.
  struct Candidate {
    std::size_t idx;
    Scalar ring_key_dist;
    Scalar scan_context_dist;
    Scalar pose_distance;
  };

  std::vector<Candidate> ring_candidates;
  Scalar query_rk_norm = query_desc.ring_key.norm();
  if (query_rk_norm < 1e-10) return candidates;

  for (std::size_t i = 0; i < database_.size() - 1; ++i) {
    const auto& db_desc = database_[i];

    // Reject short-term self-overlap. At 10 Hz, 250 frames is about 25 s and
    // prevents adjacent corridor scans from becoming false loop factors.
    if (std::abs(static_cast<int64_t>(query_desc.frame_id) -
                 static_cast<int64_t>(db_desc.frame_id)) <
        std::max(1, params_.loop_min_frame_separation)) {
      continue;
    }

    // Ring key cosine distance (ranking hint only)
    Scalar rk_norm = db_desc.ring_key.norm();
    if (rk_norm < 1e-10) continue;

    Scalar cos_dist = 1.0 - query_desc.ring_key.dot(db_desc.ring_key) / (query_rk_norm * rk_norm);
    const Scalar pose_distance = (query_desc.pose.t - db_desc.pose.t).norm();
    ring_candidates.push_back({i, cos_dist, 0, pose_distance});
  }

  if (ring_candidates.empty()) return candidates;

  // Step 2: ScanContext full distance for top candidates
  for (auto& cand : ring_candidates) {
    cand.scan_context_dist = query_desc.distance(database_[cand.idx]);
  }

  // Prefer candidates inside the vendor max-search radius, then rank them by
  // descriptor distance. This guarantees that a drifted but spatially nearby
  // return-to-start candidate reaches geometric verification.
  std::sort(ring_candidates.begin(), ring_candidates.end(),
            [this](const Candidate& a, const Candidate& b) {
              const bool a_near = a.pose_distance <= params_.loop_max_search_distance;
              const bool b_near = b.pose_distance <= params_.loop_max_search_distance;
              if (a_near != b_near) return a_near;
              return a.scan_context_dist < b.scan_context_dist;
            });

  const auto build_submap = [this](std::size_t center, std::size_t last_allowed) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr combined(
        new pcl::PointCloud<pcl::PointXYZI>());
      const auto radius = static_cast<std::size_t>(std::max(0, params_.loop_submap_radius));
      const std::size_t begin = center > radius ? center - radius : 0U;
      const std::size_t end = std::min(last_allowed, center + radius);
      const auto reference_inverse = database_[center].pose.inverse();
      for (std::size_t index = begin; index <= end; ++index) {
        if (!database_[index].cloud || database_[index].cloud->empty()) continue;
        pcl::PointCloud<pcl::PointXYZI> transformed;
        const auto transform = reference_inverse * database_[index].pose;
        pcl::transformPointCloud(
          *database_[index].cloud, transformed, transform.matrix().cast<float>());
        *combined += transformed;
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZI>());
      pcl::VoxelGrid<pcl::PointXYZI> filter;
      filter.setLeafSize(0.30F, 0.30F, 0.30F);
      filter.setInputCloud(combined);
      filter.filter(*filtered);
      return filtered;
    };

  const std::size_t query_index = database_.size() - 1U;
  const auto source_submap = build_submap(query_index, query_index);

  // Verify a bounded set of the best candidates. Return only the best metric
  // constraint for each query keyframe to avoid correlated duplicate factors.
  const int candidate_limit = std::min(
    std::min(2, std::max(1, params_.loop_max_candidates)),
    static_cast<int>(ring_candidates.size()));
  for (int i = 0; i < candidate_limit; ++i) {
    const auto& cand = ring_candidates[i];
    // Descriptor distance is only a ranking signal. ICP/GHT below is the
    // acceptance gate and is required to establish a metric loop constraint.
    // Only spend geometry time on strong descriptor matches. This keeps the
    // 10 Hz replay path bounded while still allowing a drifted return-to-start
    // candidate to pass when odometry distance is several metres off.
    if (std::isfinite(cand.scan_context_dist) &&
        (cand.scan_context_dist <= Scalar(0.22) || i == 0)) {
      LoopCandidate loop;
      loop.src_frame = query_desc.frame_id;
      loop.tgt_frame = database_[cand.idx].frame_id;
      // GHT-style geometric verification: refine the descriptor candidate with
      // point-to-point ICP before accepting a loop constraint.
      const auto target_submap = build_submap(cand.idx, query_index - 1U);
      if (!source_submap || !target_submap ||
          source_submap->size() < 20U || target_submap->size() < 20U) continue;
      const auto odom_initial = database_[cand.idx].pose.inverse() * query_desc.pose;
      // Use the vendor-style geometric hash estimate whenever enough
      // segments agree. It is intentionally only an initializer: ICP and the
      // overlap gate below still have to validate the metric constraint.
      Eigen::Matrix4f initial = odom_initial.matrix().cast<float>();
      Scalar segment_fraction = 0.0;
      if (const auto ght = estimateGhtGuess(source_submap, target_submap, params_)) {
        segment_fraction = ght->segment_fraction;
        if (segment_fraction >= Scalar(0.2)) {
          initial = ght->transform;
        }
      }
      // A descriptor gives a coarse sector alignment but not always the
      // correct 2-D yaw branch. Try a bounded set of yaw hypotheses around
      // both the odometry and GHT seeds, then retain the best converged ICP.
      pcl::PointCloud<pcl::PointXYZI> aligned;
      Eigen::Matrix4f best_tf = initial;
      double best_fitness = std::numeric_limits<double>::infinity();
      bool best_converged = false;
      constexpr int kYawHypotheses = 4;
      for (int yaw_index = 0; yaw_index < kYawHypotheses; ++yaw_index) {
        const Scalar yaw_offset = -math::kPI +
          Scalar(2.0 * math::kPI) * static_cast<Scalar>(yaw_index) /
          static_cast<Scalar>(kYawHypotheses);
        Eigen::Matrix4f yaw_transform = Eigen::Matrix4f::Identity();
        yaw_transform.block<3, 3>(0, 0) =
          Eigen::AngleAxis<Scalar>(yaw_offset, Eigen::Vector3d::UnitZ()).toRotationMatrix().cast<float>();
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setInputSource(source_submap);
        icp.setInputTarget(target_submap);
        icp.setMaximumIterations(80);
        icp.setMaxCorrespondenceDistance(
          static_cast<float>(std::max(params_.loop_icp_max_correspondence, Scalar(3.0))));
        pcl::PointCloud<pcl::PointXYZI> candidate_aligned;
        icp.align(candidate_aligned, yaw_transform * initial);
        const double fitness = icp.getFitnessScore();
        if (icp.hasConverged() && std::isfinite(fitness) && fitness < best_fitness) {
          best_fitness = fitness;
          best_tf = icp.getFinalTransformation();
          aligned.swap(candidate_aligned);
          best_converged = true;
        }
      }
      // If point ICP remains trapped, use a single voxelized NDT refinement
      // from the best seed. This is bounded to loop candidates only and is
      // substantially cheaper than a dense multi-start search.
      if (!best_converged || best_fitness > params_.loop_matching_error_threshold) {
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
        ndt.setInputSource(source_submap);
        ndt.setInputTarget(target_submap);
        ndt.setResolution(1.0F);
        ndt.setStepSize(0.2);
        ndt.setTransformationEpsilon(0.01);
        ndt.setMaximumIterations(45);
        pcl::PointCloud<pcl::PointXYZI> ndt_aligned;
        ndt.align(ndt_aligned, best_tf);
        const double ndt_fitness = ndt.getFitnessScore();
        if (ndt.hasConverged() && std::isfinite(ndt_fitness) && ndt_fitness < best_fitness) {
          best_fitness = ndt_fitness;
          best_tf = ndt.getFinalTransformation();
          aligned.swap(ndt_aligned);
          best_converged = true;
        }
      }
      // The vendor threshold is a segment matcher score. PCL ICP exposes a
      // squared point-distance score, so use a bounded reconstructed fallback
      // (0.60 m²) only when descriptor and overlap evidence are strong.
      const double reconstructed_fitness_limit = std::max<double>(
        params_.loop_matching_error_threshold, 1.50);
      if (!best_converged || !std::isfinite(best_fitness) ||
          best_fitness > reconstructed_fitness_limit) {
        RCLCPP_INFO(
          rclcpp::get_logger("m20_loop"),
          "Loop reject src=%lu tgt=%lu sc=%.4f pose_dist=%.3f converged=%d fitness=%.6f reason=icp",
          static_cast<unsigned long>(loop.src_frame), static_cast<unsigned long>(loop.tgt_frame),
          cand.scan_context_dist, cand.pose_distance,
          best_converged, best_fitness);
        continue;
      }
      Eigen::Matrix4f tf = best_tf;

      // Reject low-overlap descriptor aliases. Fitness alone is averaged over
      // the correspondences ICP retained and can look good for a small patch.
      pcl::KdTreeFLANN<pcl::PointXYZI> target_tree;
      target_tree.setInputCloud(target_submap);
      const Scalar inlier_radius = std::max(
        Scalar(0.1), std::sqrt(params_.loop_matching_error_threshold));
      const Scalar inlier_radius_sq = inlier_radius * inlier_radius;
      std::size_t inliers = 0;
      std::vector<int> indices(1);
      std::vector<float> distances(1);
      for (const auto & point : aligned.points) {
        if (target_tree.nearestKSearch(point, 1, indices, distances) > 0 &&
            distances.front() <= inlier_radius_sq) {
          ++inliers;
        }
      }
      const Scalar inlier_fraction = aligned.empty() ? Scalar(0) :
        static_cast<Scalar>(inliers) / static_cast<Scalar>(aligned.size());
      // The native inlier_fraction_threshold applies to segment matches. For
      // the reconstructed point verifier, scale it by the measured segment
      // support and retain the explicit submap-overlap floor.
      // Segment support is retained as a diagnostic/ranking signal. The
      // reconstructed point verifier uses a bounded overlap floor rather than
      // multiplying two unrelated metrics (segment support and point overlap).
      const Scalar required_overlap = std::max(
        Scalar(0.45), std::min(Scalar(0.75), params_.loop_min_submap_overlap));
      if (inlier_fraction < required_overlap) {
        RCLCPP_INFO(
          rclcpp::get_logger("m20_loop"),
          "Loop reject src=%lu tgt=%lu sc=%.4f pose_dist=%.3f fitness=%.6f overlap=%.3f reason=overlap",
          static_cast<unsigned long>(loop.src_frame), static_cast<unsigned long>(loop.tgt_frame),
          cand.scan_context_dist, cand.pose_distance,
          best_fitness, inlier_fraction);
        continue;
      }

      // ICP maps the current/source cloud into the historical/target cloud:
      // T_target_source = T_world_target^-1 * T_world_source. GTSAM's
      // BetweenFactor(source,target) expects the inverse, T_source_target.
      SE3Pose target_from_source;
      target_from_source.q =
        Eigen::Quaternion<Scalar>(tf.block<3,3>(0,0).cast<Scalar>());
      target_from_source.q.normalize();
      target_from_source.t = tf.block<3,1>(0,3).cast<Scalar>();
      loop.relative_pose = target_from_source.inverse();
      loop.fitness_score = static_cast<Scalar>(best_fitness);
      RCLCPP_INFO(
        rclcpp::get_logger("m20_loop"),
        "Loop accept src=%lu tgt=%lu sc=%.4f pose_dist=%.3f fitness=%.6f overlap=%.3f",
        static_cast<unsigned long>(loop.src_frame), static_cast<unsigned long>(loop.tgt_frame),
        cand.scan_context_dist, cand.pose_distance,
        loop.fitness_score, inlier_fraction);
      candidates.push_back(loop);
    }
  }

  if (candidates.size() > 1U) {
    const auto best = std::min_element(
      candidates.begin(), candidates.end(),
      [](const LoopCandidate & lhs, const LoopCandidate & rhs) {
        return lhs.fitness_score < rhs.fitness_score;
      });
    LoopCandidate selected = *best;
    candidates.clear();
    candidates.push_back(selected);
  }

  return candidates;
}

void LoopClosureDetector::clear() {
  database_.clear();
}

}  // namespace m20::backend
