#include "m20_slam_navigation/ros/drdds_pointcloud_source.hpp"
#include "m20_slam_navigation/ros/drdds_imu_source.hpp"
#include "m20_slam_navigation/ros/pointcloud_wire.hpp"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

std::atomic<bool> running{true};

void signalHandler(int)
{
  running = false;
}

bool writeExact(int fd, const void * input, std::size_t size)
{
  const auto * bytes = static_cast<const std::uint8_t *>(input);
  std::size_t completed = 0;
  while (completed < size && running) {
    const ssize_t result = ::send(fd, bytes + completed, size - completed, MSG_NOSIGNAL);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    completed += static_cast<std::size_t>(result);
  }
  return completed == size;
}

class PacketSocketServer
{
public:
  PacketSocketServer(std::string path, std::uint32_t magic)
  : path_(std::move(path)), magic_(magic) {}

  ~PacketSocketServer()
  {
    stop();
  }

  bool start(std::string & error)
  {
    server_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      error = std::strerror(errno);
      return false;
    }
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path_.size() >= sizeof(address.sun_path)) {
      error = "socket path is too long";
      return false;
    }
    std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1);
    ::unlink(path_.c_str());
    if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      ::listen(server_fd_, 1) != 0)
    {
      error = std::strerror(errno);
      return false;
    }
    if (::chmod(path_.c_str(), 0666) != 0) {
      error = std::strerror(errno);
      return false;
    }
    worker_ = std::thread([this]() {run();});
    return true;
  }

  void submit(std::vector<std::uint8_t> payload)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(payload);
    ++received_;
    condition_.notify_one();
  }

  std::uint64_t received() const {return received_.load();}
  std::uint64_t sent() const {return sent_.load();}

private:
  void stop()
  {
    stopped_ = true;
    condition_.notify_all();
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);
      ::close(server_fd_);
      server_fd_ = -1;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    ::unlink(path_.c_str());
  }

  void run()
  {
    while (!stopped_ && running) {
      const int client = ::accept(server_fd_, nullptr, nullptr);
      if (client < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      while (!stopped_ && running) {
        std::vector<std::uint8_t> payload;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          condition_.wait_for(lock, std::chrono::milliseconds(500), [this]() {
            return stopped_ || latest_.has_value();
          });
          if (!latest_) {
            continue;
          }
          payload = std::move(*latest_);
          latest_.reset();
        }
        const std::uint32_t header[2]{
          magic_, static_cast<std::uint32_t>(payload.size())};
        if (!writeExact(client, header, sizeof(header)) ||
          !writeExact(client, payload.data(), payload.size()))
        {
          break;
        }
        ++sent_;
      }
      ::close(client);
    }
  }

  std::string path_;
  std::uint32_t magic_{0};
  int server_fd_{-1};
  std::atomic<bool> stopped_{false};
  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> sent_{0};
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<std::vector<std::uint8_t>> latest_;
  std::thread worker_;
};

}  // namespace

int main(int argc, char ** argv)
{
  m20::ros::DrddsPointCloudSourceOptions lidar_options;
  m20::ros::DrddsImuSourceOptions imu_options;
  std::string lidar_socket_path = "/tmp/m20_drdds_lidar.sock";
  std::string imu_socket_path = "/tmp/m20_drdds_imu.sock";
  bool enable_imu = true;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if ((argument == "--topic" || argument == "--lidar-topic") && index + 1 < argc) {
      lidar_options.topic = argv[++index];
    } else if (argument == "--imu-topic" && index + 1 < argc) {
      imu_options.topic = argv[++index];
    } else if (argument == "--domain" && index + 1 < argc) {
      lidar_options.domain_id = imu_options.domain_id = std::stoi(argv[++index]);
    } else if (argument == "--prefix" && index + 1 < argc) {
      lidar_options.topic_prefix = imu_options.topic_prefix = argv[++index];
    } else if (argument == "--network" && index + 1 < argc) {
      lidar_options.network_name = imu_options.network_name = argv[++index];
    } else if ((argument == "--socket" || argument == "--lidar-socket") && index + 1 < argc) {
      lidar_socket_path = argv[++index];
    } else if (argument == "--imu-socket" && index + 1 < argc) {
      imu_socket_path = argv[++index];
    } else if (argument == "--shm") {
      lidar_options.use_shm = imu_options.use_shm = true;
    } else if (argument == "--no-imu") {
      enable_imu = false;
    } else {
      std::cerr << "Unknown or incomplete argument: " << argument << '\n';
      return 2;
    }
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  PacketSocketServer lidar_server(lidar_socket_path, m20::ros::kPointCloudWireMagic);
  PacketSocketServer imu_server(imu_socket_path, m20::ros::kImuWireMagic);
  std::string error;
  if (!lidar_server.start(error)) {
    std::cerr << "Cannot start point-cloud socket server: " << error << '\n';
    return 1;
  }
  if (enable_imu && !imu_server.start(error)) {
    std::cerr << "Cannot start IMU socket server: " << error << '\n';
    return 1;
  }
  auto lidar_source = m20::ros::DrddsPointCloudSource::create(
    lidar_options,
    [&lidar_server](m20::ros::DrddsPointCloud && cloud) {
      lidar_server.submit(m20::ros::serializePointCloud(cloud));
    }, error);
  if (!lidar_source) {
    std::cerr << "Cannot subscribe to vendor DrDDS cloud: " << error << '\n';
    return 1;
  }
  std::unique_ptr<m20::ros::DrddsImuSource> imu_source;
  if (enable_imu) {
    imu_source = m20::ros::DrddsImuSource::create(
      imu_options,
      [&imu_server](m20::ros::DrddsImu && imu) {
        imu_server.submit(m20::ros::serializeImu(imu));
      }, error);
    if (!imu_source) {
      std::cerr << "Cannot subscribe to vendor DrDDS IMU: " << error << '\n';
      return 1;
    }
  }

  std::cout << "DrDDS vendor sensor gateway started: lidar=" << lidar_options.topic
            << " imu=" << (enable_imu ? imu_options.topic : "disabled")
            << " domain=" << lidar_options.domain_id << " prefix=" << lidar_options.topic_prefix
            << " lidar_socket=" << lidar_socket_path
            << " imu_socket=" << (enable_imu ? imu_socket_path : "disabled") << std::endl;
  while (running) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "DrDDS status: cloud(matched=" << lidar_source->matchedPublishers()
              << " updated=" << (lidar_source->updatedWithin(2500) ? "yes" : "no")
              << " received=" << lidar_server.received() << " forwarded=" << lidar_server.sent()
              << ")";
    if (imu_source) {
      std::cout << " imu(matched=" << imu_source->matchedPublishers()
                << " updated=" << (imu_source->updatedWithin(2500) ? "yes" : "no")
                << " received=" << imu_server.received() << " forwarded=" << imu_server.sent()
                << ")";
    }
    std::cout << std::endl;
  }
  return 0;
}
