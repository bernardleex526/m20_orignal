#include "m20_slam_navigation/ros/native_navigation_bridge.hpp"

#include "m20_slam_navigation/control/dwa_planner.hpp"

#include <cmath>
#include <utility>

#ifdef M20_HAS_DRDDS

#include <drdds/core/drdds_core.h>
#include <dridl/dr_msgs/msg/MotionInfoPubSubTypes.h>
#include <dridl/dr_msgs/msg/NavCmdPubSubTypes.h>
#include <dridl/dr_msgs/msg/PlannerStatusPubSubTypes.h>
#include <dridl/dr_msgs/msg/StdMsgInt32PubSubTypes.h>
#include <dridl/dr_srvs/srv/PoseStampedToInt32PubSubTypes.h>
#include <dridl/dr_srvs/srv/StdSrvInt32PubSubTypes.h>
#include <dridl/dr_srvs/srv/set_paramPubSubTypes.h>

#include <exception>

#endif

namespace m20::ros {

struct NativeNavigationBridge::Impl {
  NativeNavigationCallbacks callbacks;
  bool active{false};

#ifdef M20_HAS_DRDDS
  using NavCmdChannel = DrDDSChannel<drdds::msg::NavCmdPubSubType>;
  using PlannerStatusChannel = DrDDSChannel<drdds::msg::PlannerStatusPubSubType>;
  using MotionInfoChannel = DrDDSChannel<drdds::msg::MotionInfoPubSubType>;
  using PlannerModeChannel = DrDDSChannel<drdds::msg::StdMsgInt32PubSubType>;
  using GoalRequest = drdds::srv::PoseStampedToInt32_RequestPubSubType;
  using GoalResponse = drdds::srv::PoseStampedToInt32_ResponsePubSubType;
  using CancelRequest = drdds::srv::StdSrvInt32_RequestPubSubType;
  using CancelResponse = drdds::srv::StdSrvInt32_ResponsePubSubType;
  using SetParamRequest = drdds::srv::SetParam_RequestPubSubType;
  using SetParamResponse = drdds::srv::SetParam_ResponsePubSubType;

  std::unique_ptr<NavCmdChannel> nav_cmd;
  std::unique_ptr<PlannerStatusChannel> global_status;
  std::unique_ptr<PlannerStatusChannel> local_status;
  std::unique_ptr<MotionInfoChannel> motion_info;
  std::unique_ptr<PlannerModeChannel> planner_mode;
  std::unique_ptr<DrDDSServerChannel<GoalRequest, GoalResponse>> goal_global;
  std::unique_ptr<DrDDSServerChannel<GoalRequest, GoalResponse>> goal_planner;
  std::unique_ptr<DrDDSServerChannel<CancelRequest, CancelResponse>> cancel_global;
  std::unique_ptr<DrDDSServerChannel<CancelRequest, CancelResponse>> cancel_planner;
  std::unique_ptr<DrDDSServerChannel<SetParamRequest, SetParamResponse>> set_param;
#endif
};

NativeNavigationBridge::NativeNavigationBridge()
    : impl_(std::make_unique<Impl>()) {}

NativeNavigationBridge::~NativeNavigationBridge() {
  stop();
}

bool NativeNavigationBridge::start(
    const native::DDSOptions& options,
    const m20::NavigationTopics& topics,
    NativeNavigationCallbacks callbacks,
    std::string& error) {
  stop();
  impl_->callbacks = std::move(callbacks);

#ifdef M20_HAS_DRDDS
  try {
    DrDDSManager::Init(options.domain_id, options.network_name);

    impl_->nav_cmd = std::make_unique<Impl::NavCmdChannel>(
        topics.nav_cmd, options.domain_id, options.use_shm, options.topic_prefix);
    impl_->global_status = std::make_unique<Impl::PlannerStatusChannel>(
        topics.global_planner_status, options.domain_id, options.use_shm, options.topic_prefix);
    impl_->local_status = std::make_unique<Impl::PlannerStatusChannel>(
        topics.planner_status, options.domain_id, options.use_shm, options.topic_prefix);

    impl_->motion_info = std::make_unique<Impl::MotionInfoChannel>(
        [this](const drdds::msg::MotionInfo* message) {
          if (message && impl_->callbacks.motion_info) {
            const auto& data = message->data();
            impl_->callbacks.motion_info(
                data.vel_x(), data.vel_y(), data.vel_yaw());
          }
        }, topics.motion_info,
        options.domain_id, options.use_shm, options.topic_prefix);
    impl_->planner_mode = std::make_unique<Impl::PlannerModeChannel>(
        [this](const drdds::msg::StdMsgInt32* message) {
          if (message && impl_->callbacks.planner_mode) {
            impl_->callbacks.planner_mode(message->value());
          }
        }, topics.planner_mode, options.domain_id, options.use_shm, options.topic_prefix);

    const auto goal_handler = [this](
        const drdds::srv::PoseStampedToInt32_Request* request,
        drdds::srv::PoseStampedToInt32_Response* response) {
      if (request && impl_->callbacks.goal) {
        const auto& pose_stamped = request->pose();
        const auto& position = pose_stamped.pose().position();
        const auto& orientation = pose_stamped.pose().orientation();
        const double yaw = std::atan2(
            2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
            1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
        const bool accepted = impl_->callbacks.goal(
            position.x(), position.y(), yaw, pose_stamped.header().frame_id());
        if (response) response->result(accepted);
      } else if (response) {
        response->result(false);
      }
    };

    const auto cancel_handler = [this](
        const drdds::srv::StdSrvInt32_Request* request,
        drdds::srv::StdSrvInt32_Response* response) {
      if (request && impl_->callbacks.cancel) {
        impl_->callbacks.cancel(request->command());
        if (response) response->result(1);
      } else if (response) {
        response->result(0);
      }
    };

    const auto set_param_handler = [this](
        const drdds::srv::SetParam_Request* request,
        drdds::srv::SetParam_Response* response) {
      if (request && impl_->callbacks.set_param) {
        native::LocalParamUpdate update;
        update.directline_mode = request->directLine_mode();
        update.max_speed_x = request->maxSpeedX();
        update.max_speed_y = request->maxSpeedY();
        update.max_theta = request->maxTheta();
        update.barking_deceleration = request->barking_deceleration();
        update.xy_tolerance = request->xy_tolerance();
        update.yaw_tolerance = request->yaw_tolerance();
        update.stop_distance = request->stop_distance();
        update.judge_close = request->judge_close() != 0.0;
        update.backward_mode = request->backward_mode() != 0.0;
        update.dl_min_yaw = request->dl_min_yaw();
        update.dl_min_x = request->dl_min_x();
        update.dl_min_y = request->dl_min_y();
        update.dl_line_yaw_min = request->dl_line_yaw_min();
        impl_->callbacks.set_param(update);
        if (response) response->is_success(true);
      } else if (response) {
        response->is_success(false);
      }
    };

    impl_->goal_global = std::make_unique<DrDDSServerChannel<Impl::GoalRequest, Impl::GoalResponse>>(
        goal_handler, topics.goal_global_service, options.network_name, options.domain_id);
    impl_->goal_planner = std::make_unique<DrDDSServerChannel<Impl::GoalRequest, Impl::GoalResponse>>(
        goal_handler, topics.goal_planner_service, options.network_name, options.domain_id);
    impl_->cancel_global = std::make_unique<DrDDSServerChannel<Impl::CancelRequest, Impl::CancelResponse>>(
        cancel_handler, topics.cancel_global_service, options.network_name, options.domain_id);
    impl_->cancel_planner = std::make_unique<DrDDSServerChannel<Impl::CancelRequest, Impl::CancelResponse>>(
        cancel_handler, topics.cancel_planner_service, options.network_name, options.domain_id);
    impl_->set_param = std::make_unique<DrDDSServerChannel<Impl::SetParamRequest, Impl::SetParamResponse>>(
        set_param_handler, topics.set_param_service, options.network_name, options.domain_id);

    impl_->active = true;
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
  } catch (...) {
    error = "unknown native DrDDS navigation initialization failure";
  }
  stop();
  return false;
#else
  static_cast<void>(options);
  static_cast<void>(topics);
  error = "m20_slam_navigation was built without the vendor DrDDS development package";
  return false;
#endif
}

void NativeNavigationBridge::stop() {
#ifdef M20_HAS_DRDDS
  if (impl_) {
    impl_->set_param.reset();
    impl_->cancel_planner.reset();
    impl_->cancel_global.reset();
    impl_->goal_planner.reset();
    impl_->goal_global.reset();
    impl_->planner_mode.reset();
    impl_->motion_info.reset();
    impl_->local_status.reset();
    impl_->global_status.reset();
    impl_->nav_cmd.reset();
    if (impl_->active) {
      DrDDSManager::Delete();
      impl_->active = false;
    }
  }
#else
  if (impl_) impl_->active = false;
#endif
}

bool NativeNavigationBridge::available() const {
  return impl_ && impl_->active;
}

bool NativeNavigationBridge::publishNavCmd(const m20::control::VelocityCommand& command) {
#ifdef M20_HAS_DRDDS
  if (!available() || !impl_->nav_cmd) return false;
  drdds::msg::NavCmdValue data;
  data.x_vel(static_cast<float>(command.vx));
  data.y_vel(static_cast<float>(command.vy));
  data.yaw_vel(static_cast<float>(command.omega));
  drdds::msg::NavCmd message;
  message.data(data);
  return impl_->nav_cmd->Write(&message);
#else
  static_cast<void>(command);
  return false;
#endif
}

bool NativeNavigationBridge::publishPlannerStatus(
    bool global, std::int32_t status, std::int32_t warning_state) {
#ifdef M20_HAS_DRDDS
  if (!available()) return false;
  auto* channel = global ? impl_->global_status.get() : impl_->local_status.get();
  if (!channel) return false;
  drdds::msg::PlannerStatusValue data;
  data.status(status);
  data.warning_state(warning_state);
  drdds::msg::PlannerStatus message;
  message.data(data);
  return channel->Write(&message);
#else
  static_cast<void>(global);
  static_cast<void>(status);
  static_cast<void>(warning_state);
  return false;
#endif
}

}  // namespace m20::ros
