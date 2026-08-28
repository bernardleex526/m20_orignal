#pragma once

namespace m20::ros {

struct VendorAuxiliaryOutputPolicy {
  bool publish_depth_cloud{false};
  bool publish_accumulated_cloud{true};
  bool populate_accumulated_cloud{false};
};

inline VendorAuxiliaryOutputPolicy vendorAuxiliaryOutputPolicy(
  bool ray_casting_enabled, bool accumulated_points_enabled) noexcept
{
  return {
    ray_casting_enabled,
    true,
    accumulated_points_enabled,
  };
}

}  // namespace m20::ros
