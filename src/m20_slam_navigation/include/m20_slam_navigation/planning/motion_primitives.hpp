#pragma once
/**
 * @file motion_primitives.hpp
 * @brief Omnidirectional motion primitives for hybrid A* on quadruped robots.
 *
 * Unlike wheeled vehicles, quadruped robots are holonomic/omnidirectional,
 * enabling:
 *   - Forward/backward translation
 *   - Lateral (sideways) sliding
 *   - Diagonal motion
 *   - Rotation-in-place (yaw turn)
 *   - Combined translation + rotation
 *
 * Primitive set is defined as 8-directional translation combined with
 * 3 heading changes (straight, left turn, right turn), plus rotation-in-place,
 * yielding 8×3 + 1 = 25 primitives (configurable).
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <Eigen/Dense>

#include <vector>

namespace m20::planning {

class MotionPrimitives {
public:
  /**
   * @brief Generate omnidirectional motion primitive set.
   *
   * @param num_angles      Number of discretized translation directions (e.g. 8)
   * @param primitive_len   Arc/step length per primitive [m]
   * @param max_curvature   Max path curvature κ_max for rotation primitives [1/m]
   * @param num_rotations   Number of heading-change options per translation (e.g. 3: straight, left, right)
   * @return                Vector of motion primitives
   */
  static std::vector<MotionPrimitive> generateOmnidirectional(
      int num_angles = 8,
      Scalar primitive_len = 0.3,
      Scalar max_curvature = 0.5,
      int num_rotations = 3);

  /**
   * @brief Generate rotation-in-place primitives.
   *
   * @param num_headings   Number of discrete heading changes
   * @param max_angle      Max rotation per step [rad]
   * @return               Rotation-only primitives
   */
  static std::vector<MotionPrimitive> generateRotationInPlace(
      int num_headings = 3,
      Scalar max_angle = 0.3);

  /**
   * @brief Apply a motion primitive to a state [x, y, θ].
   *
   * New state: x' = x + dx·cos(θ) − dy·sin(θ)
   *            y' = y + dx·sin(θ) + dy·cos(θ)
   *            θ' = normalize_angle(θ + dθ)
   */
  static void apply(const MotionPrimitive& mp,
                    Scalar x, Scalar y, Scalar theta,
                    Scalar& x_out, Scalar& y_out, Scalar& theta_out);

  /// Check if primitive is valid at a given cell (collision-free, within bounds)
  static bool isValid(const MotionPrimitive& mp,
                      Scalar x, Scalar y, Scalar theta,
                      const std::vector<uint8_t>& costmap,
                      int width, int height, Scalar resolution,
                      uint8_t lethal_threshold = 254);
};

}  // namespace m20::planning