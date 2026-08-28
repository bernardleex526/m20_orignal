#pragma once
/**
 * @file types.hpp
 * @brief Core type definitions for the M20 quadruped SLAM & navigation system.
 *
 * Defines fundamental Lie algebra wrappers (SE(3) / SO(3)), point cloud
 * structures, sensor packet types, and algorithm configuration enumerations.
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace m20 {

// =============================================================================
// Timestamp & index aliases
// =============================================================================
using Timestamp = std::chrono::time_point<std::chrono::steady_clock>;
using FrameId   = uint64_t;
static constexpr FrameId INVALID_FRAME_ID = 0;

// =============================================================================
// Scalar types — float for embedded performance, double where precision needed
// =============================================================================
using float32_t = float;
using float64_t = double;
// Primary working scalar (tunable)
using Scalar = float64_t;

// =============================================================================
// SE(3) Pose: rotation R ∈ SO(3) + translation t ∈ ℝ³
// Stored as 4×4 homogeneous transform for efficient composition:
//   T = [ R  t ]
//       [ 0  1 ]
// =============================================================================
struct SE3Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaternion<Scalar> q{Eigen::Quaternion<Scalar>::Identity()};  ///< unit quaternion (w,x,y,z)
  Eigen::Matrix<Scalar, 3, 1> t{0.0, 0.0, 0.0};                       ///< translation [m]

  /// Construct identity
  static SE3Pose Identity() { return {}; }

  /// Homogeneous 4×4 matrix
  Eigen::Matrix<Scalar, 4, 4> matrix() const {
    Eigen::Matrix<Scalar, 4, 4> T = Eigen::Matrix<Scalar, 4, 4>::Identity();
    T.template block<3, 3>(0, 0) = q.toRotationMatrix();
    T.template block<3, 1>(0, 3) = t;
    return T;
  }

  /// Compose: this * other (i.e. T_this ∘ T_other)
  SE3Pose operator*(const SE3Pose& other) const {
    SE3Pose out;
    out.q = q * other.q;                       // quaternion composition
    out.t = t + q._transformVector(other.t);   // rotate then translate
    return out;
  }

  /// Inverse transform
  SE3Pose inverse() const {
    SE3Pose out;
    out.q = q.conjugate();
    out.t = -(out.q._transformVector(t));
    return out;
  }

  /// Transform a 3D point
  Eigen::Matrix<Scalar, 3, 1> transformPoint(const Eigen::Matrix<Scalar, 3, 1>& p) const {
    return q._transformVector(p) + t;
  }

  /// Log-map to Lie algebra se(3) twist: [ω, v] (6×1)
  Eigen::Matrix<Scalar, 6, 1> log() const;

  /// Exponential map from se(3) twist to SE(3)
  static SE3Pose exp(const Eigen::Matrix<Scalar, 6, 1>& xi);
};

// =============================================================================
// 6-DOF twist: angular velocity ω + linear velocity v
// =============================================================================
struct Twist {
  Eigen::Matrix<Scalar, 3, 1> angular{0, 0, 0};   ///< ω [rad/s]
  Eigen::Matrix<Scalar, 3, 1> linear{0, 0, 0};     ///< v [m/s]
};

// =============================================================================
// IMU measurement at a discrete timestamp
// =============================================================================
struct alignas(32) ImuPacket {
  Timestamp stamp;
  Eigen::Matrix<Scalar, 3, 1> accel{0, 0, 0};         ///< linear acceleration [m/s²] (body frame)
  Eigen::Matrix<Scalar, 3, 1> gyro{0, 0, 0};           ///< angular velocity        [rad/s] (body frame)
  Eigen::Matrix<Scalar, 3, 1> gravity{0, 0, -9.81007}; ///< pre-computed gravity vector (world frame, z-up)
};

// =============================================================================
// LiDAR scan descriptor for a single ring/scans
// =============================================================================
struct LiDARPacket {
  Timestamp stamp;       ///< scan start
  Timestamp scan_end;    ///< scan end from per-point timestamps
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;  ///< raw (possibly distorted) point cloud
  std::vector<double> point_time_offsets;       ///< seconds from scan start, one per point
  std::vector<std::uint16_t> rings;             ///< vendor ring, one per point
  FrameId frame_id{INVALID_FRAME_ID};
};

// =============================================================================
// Deskewed (motion-compensated) point cloud
// =============================================================================
struct DeskewedCloud {
  Timestamp stamp;
  FrameId frame_id{INVALID_FRAME_ID};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  SE3Pose pose_estimate;  ///< prior pose from IMU integration at this frame
};

// =============================================================================
// Foot odometry message — quadruped leg kinematics (AOS-style)
// =============================================================================
struct alignas(32) FootOdomPacket {
  Timestamp stamp;
  SE3Pose   pose;            ///< body pose in odom frame from leg kinematics
  Twist     twist;           ///< body velocity
  Eigen::Matrix<Scalar, 6, 6> covariance; ///< 6×6 pose covariance
  bool      feet_in_contact[4]{true, true, true, true}; ///< swing/contact per leg
};

// =============================================================================
// Voxel entry for incremental hash-based voxel map
// =============================================================================
struct VoxelEntry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<Scalar, 3, 1> centroid{Eigen::Matrix<Scalar, 3, 1>::Zero()};   ///< μ
  Eigen::Matrix<Scalar, 3, 3> covariance{Eigen::Matrix<Scalar, 3, 3>::Zero()}; ///< Σ
  Eigen::Matrix<Scalar, 3, 1> plane_normal{Eigen::Matrix<Scalar, 3, 1>::Zero()};
  Scalar                      plane_offset{0.0};
  uint32_t                    point_count{0};
  bool                        plane_valid{false};

  /// Online update: add point p to Gaussian distribution (Welford-like)
  void addPoint(const Eigen::Matrix<Scalar, 3, 1>& p);
  void updatePlane(Scalar eigenvalue_ratio = 0.1);
};

// =============================================================================
// Grid map cell for 2.5D elevation / traversability
// =============================================================================
struct ElevationCell {
  Scalar   min_z{1e9};
  Scalar   max_z{-1e9};
  Scalar   mean_z{0.0};
  Scalar   var_z{0.0};
  Scalar   slope{0.0};        ///< local surface slope angle [rad]
  Scalar   roughness{0.0};    ///< σ_z
  Scalar   step_height{0.0};  ///< max positive step diff to neighbours
  uint32_t n_points{0};
  bool     traversable{true};
  Scalar   cost{0.0};         ///< aggregated traversability cost [0, 255]

  void update(Scalar z);
};

// =============================================================================
// SE(3) pose with covariance
// =============================================================================
struct PoseWithCovariance {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SE3Pose pose;
  Eigen::Matrix<Scalar, 6, 6> covariance{Eigen::Matrix<Scalar, 6, 6>::Identity()};
};

// =============================================================================
// Loop closure candidate
// =============================================================================
struct LoopCandidate {
  FrameId src_frame;
  FrameId tgt_frame;
  SE3Pose relative_pose;
  Scalar  fitness_score;
};

// =============================================================================
// ESKF state: nominal state + error state δx
// Nominal: [p, q, v, ba, bg]  (position, orientation, velocity, accel bias, gyro bias)
// Error:   [δp, δθ, δv, δba, δbg]  (15-dim)
// =============================================================================
struct ESKFState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Nominal state
  Eigen::Matrix<Scalar, 3, 1>     p{0, 0, 0};     ///< position [m]
  Eigen::Quaternion<Scalar>       q{Eigen::Quaternion<Scalar>::Identity()}; ///< orientation
  Eigen::Matrix<Scalar, 3, 1>     v{0, 0, 0};     ///< velocity [m/s]
  Eigen::Matrix<Scalar, 3, 1>     ba{0, 0, 0};    ///< accelerometer bias
  Eigen::Matrix<Scalar, 3, 1>     bg{0, 0, 0};    ///< gyroscope bias

  // Error-state covariance 15×15
  Eigen::Matrix<Scalar, 15, 15>   P{Eigen::Matrix<Scalar, 15, 15>::Identity()};

  /// Gravity vector in world frame (z-up convention)
  inline static const Eigen::Matrix<Scalar, 3, 1> GRAVITY{0.0, 0.0, -9.81007};
};

// =============================================================================
// Planar motion primitive for hybrid A* (omnidirectional)
// =============================================================================
struct MotionPrimitive {
  Scalar dx;       ///< x displacement [m]
  Scalar dy;       ///< y displacement [m]
  Scalar dtheta;   ///< heading change [rad]
  Scalar cost;     ///< primitive cost
};

}  // namespace m20

// STL vector alignment for Eigen types
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(m20::SE3Pose)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(m20::VoxelEntry)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(m20::PoseWithCovariance)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(m20::ESKFState)
