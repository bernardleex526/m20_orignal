#include "m20_slam_navigation/ros/drdds_pointcloud_source.hpp"
#include "m20_slam_navigation/ros/drdds_imu_source.hpp"

#ifdef M20_HAS_DRDDS

#include <drdds/core/drdds_core.h>
#include <dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h>
#include <dridl/sensor_msgs/msg/ImuPubSubTypes.h>

#include <exception>
#include <mutex>
#include <utility>

namespace m20::ros
{
namespace
{

using VendorPubSubType = sensor_msgs::msg::PointCloud2PubSubType;
using VendorPointCloud = VendorPubSubType::type;
using VendorCallback = std::function<void(const VendorPointCloud *)>;

std::mutex manager_mutex;
std::size_t manager_users{0};

void acquireManager(const DrddsPointCloudSourceOptions & options)
{
  std::lock_guard<std::mutex> lock(manager_mutex);
  if (manager_users == 0) {
    DrDDSManager::Init(options.domain_id, options.network_name);
  }
  ++manager_users;
}

void releaseManager()
{
  std::lock_guard<std::mutex> lock(manager_mutex);
  if (manager_users > 0 && --manager_users == 0) {
    DrDDSManager::Delete();
  }
}

DrddsPointCloud copyCloud(const VendorPointCloud & input)
{
  DrddsPointCloud output;
  output.stamp_sec = input.header().stamp().sec();
  output.stamp_nanosec = input.header().stamp().nanosec();
  output.frame_id = input.header().frame_id();
  output.height = input.height();
  output.width = input.width();
  output.fields.reserve(input.fields().size());
  for (const auto & field : input.fields()) {
    output.fields.push_back(
      DrddsPointField{field.name(), field.offset(), field.datatype(), field.count()});
  }
  output.is_bigendian = input.is_bigendian();
  output.point_step = input.point_step();
  output.row_step = input.row_step();
  output.data = input.data();
  output.is_dense = input.is_dense();
  return output;
}

using VendorImuPubSubType = sensor_msgs::msg::ImuPubSubType;
using VendorImu = VendorImuPubSubType::type;
using VendorImuCallback = std::function<void(const VendorImu *)>;

DrddsImu copyImu(const VendorImu & input)
{
  DrddsImu output;
  output.stamp_sec = input.header().stamp().sec();
  output.stamp_nanosec = input.header().stamp().nanosec();
  output.frame_id = input.header().frame_id();
  output.orientation = {
    input.orientation().x(), input.orientation().y(), input.orientation().z(),
    input.orientation().w()};
  output.orientation_covariance = input.orientation_covariance();
  output.angular_velocity = {
    input.angular_velocity().x(), input.angular_velocity().y(), input.angular_velocity().z()};
  output.angular_velocity_covariance = input.angular_velocity_covariance();
  output.linear_acceleration = {
    input.linear_acceleration().x(), input.linear_acceleration().y(),
    input.linear_acceleration().z()};
  output.linear_acceleration_covariance = input.linear_acceleration_covariance();
  return output;
}

class VendorDrddsPointCloudSource final : public DrddsPointCloudSource
{
public:
  VendorDrddsPointCloudSource(
    const DrddsPointCloudSourceOptions & options, Callback callback)
  : callback_(std::move(callback))
  {
    acquireManager(options);
    manager_acquired_ = true;
    vendor_callback_ = [this](const VendorPointCloud * message) {
        if (message && callback_) {
          callback_(copyCloud(*message));
        }
      };
    channel_ = std::make_unique<DrDDSChannel<VendorPubSubType>>(
      vendor_callback_,
      options.topic, options.domain_id, options.use_shm, options.topic_prefix);
    if (channel_->GetSubscriber()) {
      channel_->GetSubscriber()->BindCallBack(vendor_callback_);
    }
  }

  ~VendorDrddsPointCloudSource() override
  {
    channel_.reset();
    if (manager_acquired_) {
      releaseManager();
    }
  }

  int matchedPublishers() const override
  {
    return channel_ ? channel_->GetMatchedCount() : 0;
  }

  bool updatedWithin(std::uint16_t milliseconds) const override
  {
    return channel_ ? channel_->IsUpdate(milliseconds) : false;
  }

private:
  Callback callback_;
  VendorCallback vendor_callback_;
  std::unique_ptr<DrDDSChannel<VendorPubSubType>> channel_;
  bool manager_acquired_{false};
};

class VendorDrddsImuSource final : public DrddsImuSource
{
public:
  VendorDrddsImuSource(const DrddsImuSourceOptions & options, Callback callback)
  : callback_(std::move(callback))
  {
    DrddsPointCloudSourceOptions manager_options;
    manager_options.domain_id = options.domain_id;
    manager_options.network_name = options.network_name;
    acquireManager(manager_options);
    manager_acquired_ = true;
    vendor_callback_ = [this](const VendorImu * message) {
        if (message && callback_) {
          callback_(copyImu(*message));
        }
      };
    channel_ = std::make_unique<DrDDSChannel<VendorImuPubSubType>>(
      vendor_callback_, options.topic, options.domain_id, options.use_shm, options.topic_prefix);
    if (channel_->GetSubscriber()) {
      channel_->GetSubscriber()->BindCallBack(vendor_callback_);
    }
  }

  ~VendorDrddsImuSource() override
  {
    channel_.reset();
    if (manager_acquired_) {
      releaseManager();
    }
  }

  int matchedPublishers() const override
  {
    return channel_ ? channel_->GetMatchedCount() : 0;
  }

  bool updatedWithin(std::uint16_t milliseconds) const override
  {
    return channel_ ? channel_->IsUpdate(milliseconds) : false;
  }

private:
  Callback callback_;
  VendorImuCallback vendor_callback_;
  std::unique_ptr<DrDDSChannel<VendorImuPubSubType>> channel_;
  bool manager_acquired_{false};
};

}  // namespace

std::unique_ptr<DrddsPointCloudSource> DrddsPointCloudSource::create(
  const DrddsPointCloudSourceOptions & options, Callback callback, std::string & error)
{
  try {
    auto source = std::make_unique<VendorDrddsPointCloudSource>(options, std::move(callback));
    error.clear();
    return source;
  } catch (const std::exception & exception) {
    error = exception.what();
  } catch (...) {
    error = "unknown DrDDS initialization failure";
  }
  return nullptr;
}

bool DrddsPointCloudSource::available()
{
  return true;
}

std::unique_ptr<DrddsImuSource> DrddsImuSource::create(
  const DrddsImuSourceOptions & options, Callback callback, std::string & error)
{
  try {
    auto source = std::make_unique<VendorDrddsImuSource>(options, std::move(callback));
    error.clear();
    return source;
  } catch (const std::exception & exception) {
    error = exception.what();
  } catch (...) {
    error = "unknown DrDDS IMU initialization failure";
  }
  return nullptr;
}

bool DrddsImuSource::available()
{
  return true;
}

}  // namespace m20::ros

#else

namespace m20::ros
{

std::unique_ptr<DrddsPointCloudSource> DrddsPointCloudSource::create(
  const DrddsPointCloudSourceOptions &, Callback, std::string & error)
{
  error = "m20_slam_navigation was built without the vendor DrDDS development package";
  return nullptr;
}

bool DrddsPointCloudSource::available()
{
  return false;
}

std::unique_ptr<DrddsImuSource> DrddsImuSource::create(
  const DrddsImuSourceOptions &, Callback, std::string & error)
{
  error = "m20_slam_navigation was built without the vendor DrDDS development package";
  return nullptr;
}

bool DrddsImuSource::available()
{
  return false;
}

}  // namespace m20::ros

#endif
