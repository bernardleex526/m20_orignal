#include "m20_slam_navigation/ros/socket_pointcloud_source.hpp"
#include "m20_slam_navigation/ros/socket_imu_source.hpp"

#include "m20_slam_navigation/ros/pointcloud_wire.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>
#include <utility>
#include <vector>

namespace m20::ros
{
namespace
{

bool readExact(int fd, void * output, std::size_t size)
{
  auto * bytes = static_cast<std::uint8_t *>(output);
  std::size_t completed = 0;
  while (completed < size) {
    const ssize_t result = ::recv(fd, bytes + completed, size - completed, 0);
    if (result == 0) {
      return false;
    }
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    completed += static_cast<std::size_t>(result);
  }
  return true;
}

class UnixSocketPointCloudSource final : public SocketPointCloudSource
{
public:
  UnixSocketPointCloudSource(std::string socket_path, Callback callback)
  : socket_path_(std::move(socket_path)), callback_(std::move(callback)), worker_([this]() {run();})
  {
  }

  ~UnixSocketPointCloudSource() override
  {
    stop_ = true;
    const int fd = socket_fd_.exchange(-1);
    if (fd >= 0) {
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void run()
  {
    while (!stop_) {
      const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      sockaddr_un address{};
      address.sun_family = AF_UNIX;
      if (socket_path_.size() >= sizeof(address.sun_path)) {
        ::close(fd);
        return;
      }
      std::strncpy(address.sun_path, socket_path_.c_str(), sizeof(address.sun_path) - 1);
      if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
        ::close(fd);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      socket_fd_ = fd;
      while (!stop_) {
        std::uint32_t header[2]{};
        if (!readExact(fd, header, sizeof(header)) || header[0] != kPointCloudWireMagic ||
          header[1] == 0 || header[1] > kMaxPointCloudWireBytes)
        {
          break;
        }
        std::vector<std::uint8_t> payload(header[1]);
        if (!readExact(fd, payload.data(), payload.size())) {
          break;
        }
        DrddsPointCloud cloud;
        std::string error;
        if (deserializePointCloud(payload, cloud, error) && callback_) {
          callback_(std::move(cloud));
        }
      }
      socket_fd_ = -1;
      ::close(fd);
    }
  }

  std::string socket_path_;
  Callback callback_;
  std::atomic<bool> stop_{false};
  std::atomic<int> socket_fd_{-1};
  std::thread worker_;
};

class UnixSocketImuSource final : public SocketImuSource
{
public:
  UnixSocketImuSource(std::string socket_path, Callback callback)
  : socket_path_(std::move(socket_path)), callback_(std::move(callback)), worker_([this]() {run();})
  {
  }

  ~UnixSocketImuSource() override
  {
    stop_ = true;
    const int fd = socket_fd_.exchange(-1);
    if (fd >= 0) {
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void run()
  {
    while (!stop_) {
      const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      sockaddr_un address{};
      address.sun_family = AF_UNIX;
      if (socket_path_.size() >= sizeof(address.sun_path)) {
        ::close(fd);
        return;
      }
      std::strncpy(address.sun_path, socket_path_.c_str(), sizeof(address.sun_path) - 1);
      if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
        ::close(fd);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      socket_fd_ = fd;
      while (!stop_) {
        std::uint32_t header[2]{};
        if (!readExact(fd, header, sizeof(header)) || header[0] != kImuWireMagic ||
          header[1] == 0 || header[1] > kMaxImuWireBytes)
        {
          break;
        }
        std::vector<std::uint8_t> payload(header[1]);
        if (!readExact(fd, payload.data(), payload.size())) {
          break;
        }
        DrddsImu imu;
        std::string error;
        if (deserializeImu(payload, imu, error) && callback_) {
          callback_(std::move(imu));
        }
      }
      socket_fd_ = -1;
      ::close(fd);
    }
  }

  std::string socket_path_;
  Callback callback_;
  std::atomic<bool> stop_{false};
  std::atomic<int> socket_fd_{-1};
  std::thread worker_;
};

}  // namespace

std::unique_ptr<SocketPointCloudSource> SocketPointCloudSource::create(
  const std::string & socket_path, Callback callback, std::string & error)
{
  if (socket_path.empty()) {
    error = "DrDDS socket path is empty";
    return nullptr;
  }
  error.clear();
  return std::make_unique<UnixSocketPointCloudSource>(socket_path, std::move(callback));
}

std::unique_ptr<SocketImuSource> SocketImuSource::create(
  const std::string & socket_path, Callback callback, std::string & error)
{
  if (socket_path.empty()) {
    error = "DrDDS IMU socket path is empty";
    return nullptr;
  }
  error.clear();
  return std::make_unique<UnixSocketImuSource>(socket_path, std::move(callback));
}

}  // namespace m20::ros
