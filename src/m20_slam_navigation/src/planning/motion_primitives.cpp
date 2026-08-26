#include "m20_slam_navigation/planning/motion_primitives.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <cmath>

namespace m20::planning {

std::vector<MotionPrimitive> MotionPrimitives::generateOmnidirectional(
    int num_angles, Scalar primitive_len, Scalar max_curvature, int num_rotations) {

  std::vector<MotionPrimitive> primitives;

  // Rotation-in-place primitives
  auto rot_prims = generateRotationInPlace(num_rotations);
  for (const auto& rp : rot_prims) {
    primitives.push_back(rp);
  }

  // Omnidirectional translation primitives
  Scalar angle_step = 2.0 * math::kPI / static_cast<Scalar>(num_angles);

  for (int a = 0; a < num_angles; ++a) {
    Scalar angle = static_cast<Scalar>(a) * angle_step;
    Scalar dx_base = primitive_len * std::cos(angle);
    Scalar dy_base = primitive_len * std::sin(angle);

    // For each translation direction, add rotation variants
    // Straight (dθ = 0), left turn (dθ > 0), right turn (dθ < 0)
    Scalar dtheta_max = max_curvature * primitive_len;  // κ_max · L

    for (int r = 0; r < num_rotations; ++r) {
      MotionPrimitive mp;
      mp.dx = dx_base;
      mp.dy = dy_base;

      if (num_rotations == 1) {
        mp.dtheta = 0;
      } else if (num_rotations == 3) {
        if (r == 0) mp.dtheta = 0;               // straight
        else if (r == 1) mp.dtheta = dtheta_max;  // left
        else mp.dtheta = -dtheta_max;             // right
      } else {
        Scalar frac = Scalar(r) / Scalar(num_rotations - 1);
        mp.dtheta = -dtheta_max + 2.0 * dtheta_max * frac;
      }

      // Cost = arc length + rotation penalty
      mp.cost = std::sqrt(mp.dx * mp.dx + mp.dy * mp.dy) + std::abs(mp.dtheta) * 0.5;
      primitives.push_back(mp);
    }
  }

  return primitives;
}

std::vector<MotionPrimitive> MotionPrimitives::generateRotationInPlace(
    int num_headings, Scalar max_angle) {

  std::vector<MotionPrimitive> primitives;

  for (int i = 0; i < num_headings; ++i) {
    MotionPrimitive mp;
    mp.dx = 0;
    mp.dy = 0;

    if (num_headings == 1) {
      mp.dtheta = 0;
    } else {
      Scalar frac = Scalar(i) / Scalar(num_headings - 1);
      mp.dtheta = -max_angle + 2.0 * max_angle * frac;
    }
    mp.cost = std::abs(mp.dtheta) * 1.0;  // rotation is expensive
    primitives.push_back(mp);
  }

  return primitives;
}

void MotionPrimitives::apply(
    const MotionPrimitive& mp,
    Scalar x, Scalar y, Scalar theta,
    Scalar& x_out, Scalar& y_out, Scalar& theta_out) {

  // Rotate translation by current heading
  Scalar cos_t = std::cos(theta);
  Scalar sin_t = std::sin(theta);

  x_out = x + mp.dx * cos_t - mp.dy * sin_t;
  y_out = y + mp.dx * sin_t + mp.dy * cos_t;
  theta_out = math::normalize_angle(theta + mp.dtheta);
}

bool MotionPrimitives::isValid(
    const MotionPrimitive& mp,
    Scalar x, Scalar y, Scalar theta,
    const std::vector<uint8_t>& costmap,
    int width, int height, Scalar resolution,
    uint8_t lethal_threshold) {

  // Check endpoints (simplified: just check final point)
  Scalar x_end, y_end, theta_end;
  apply(mp, x, y, theta, x_end, y_end, theta_end);

  int gx = static_cast<int>(std::floor(x_end / resolution));
  int gy = static_cast<int>(std::floor(y_end / resolution));

  if (gx < 0 || gx >= width || gy < 0 || gy >= height) return false;

  return costmap[gy * width + gx] < lethal_threshold;
}

}  // namespace m20::planning