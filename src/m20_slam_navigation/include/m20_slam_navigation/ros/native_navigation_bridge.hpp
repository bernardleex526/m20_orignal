#pragma once

#include "m20_slam_navigation/common/native_navigation.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace m20::control {
struct VelocityCommand;
}

namespace m20::ros {

/**
 * Callbacks from the native DrDDS navigation graph.
 *
 * The scalar callback signatures intentionally do not expose generated vendor
 * classes.  This keeps the rest of m20_orignal buildable on a normal ROS 2
 * workstation while the robot build can still use the exact DrDDS IDL types.
 */
struct NativeNavigationCallbacks {
  std::function<bool(double x, double y, double yaw, const std::string& frame_id)> goal;
  std::function<void(std::int32_t command)> cancel;
  std::function<void(std::int32_t mode)> planner_mode;
  std::function<void(double vx, double vy, double omega)> motion_info;
  std::function<void(const native::LocalParamUpdate& update)> set_param;
};

/**
 * Optional adapter for the vendor's DrDDS messages and services.
 *
 * On a workstation without the vendor SDK, start() returns false and the ROS
 * node uses standard-message fallbacks for offline tests.  On an M20Pro image
 * with the SDK, the adapter owns the native publishers/subscribers/services,
 * including /NAV_CMD, /MOTION_INFO, /planner_mode and the four native services.
 */
class NativeNavigationBridge {
public:
  NativeNavigationBridge();
  ~NativeNavigationBridge();

  NativeNavigationBridge(const NativeNavigationBridge&) = delete;
  NativeNavigationBridge& operator=(const NativeNavigationBridge&) = delete;

  bool start(const native::DDSOptions& options,
             const m20::NavigationTopics& topics,
             NativeNavigationCallbacks callbacks,
             std::string& error);
  void stop();

  bool available() const;
  bool publishNavCmd(const m20::control::VelocityCommand& command);
  bool publishPlannerStatus(bool global, std::int32_t status, std::int32_t warning_state = 0);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace m20::ros
