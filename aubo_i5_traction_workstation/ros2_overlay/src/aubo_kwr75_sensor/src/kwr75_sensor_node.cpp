#include "aubo_kwr75_sensor/kwr75_protocol.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace aubo_kwr75_sensor
{
namespace
{

constexpr std::chrono::milliseconds kReconnectDelay{1000};
constexpr std::chrono::milliseconds kReadPollTimeout{100};

speed_t baud_to_termios(const int baud_rate)
{
  switch (baud_rate) {
    case 460800:
      return B460800;
    default:
      throw std::invalid_argument("Unsupported baud_rate; KWR75D requires 460800 baud");
  }
}

bool write_all(const int file_descriptor, const std::array<std::uint8_t, 4U> & command)
{
  std::size_t written = 0U;
  while (written < command.size()) {
    const ssize_t result = write(
      file_descriptor, command.data() + written, command.size() - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return tcdrain(file_descriptor) == 0;
}

int open_serial_8n1(const std::string & serial_path, const int baud_rate)
{
  const int file_descriptor = open(serial_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (file_descriptor < 0) {
    return -1;
  }

  termios options{};
  if (tcgetattr(file_descriptor, &options) != 0) {
    close(file_descriptor);
    return -1;
  }
  cfmakeraw(&options);
  options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD | CS8);
  options.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CRTSCTS));
  options.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  const speed_t speed = baud_to_termios(baud_rate);
  if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0 ||
    tcsetattr(file_descriptor, TCSANOW, &options) != 0)
  {
    close(file_descriptor);
    return -1;
  }
  tcflush(file_descriptor, TCIOFLUSH);
  return file_descriptor;
}

}  // namespace

class Kwr75SensorNode : public rclcpp::Node
{
public:
  Kwr75SensorNode()
  : Node("kwr75_sensor_node")
  {
    serial_path_ = declare_parameter<std::string>("serial_path", "/dev/ttyUSB0");
    const std::int64_t baud_rate = declare_parameter<std::int64_t>("baud_rate", 460800);
    frame_id_ = declare_parameter<std::string>("frame_id", "kwr75_sensor");
    start_stream_on_connect_ = declare_parameter<bool>("start_stream_on_connect", true);
    stop_stream_on_shutdown_ = declare_parameter<bool>("stop_stream_on_shutdown", true);
    const std::int64_t stale_timeout_ms = declare_parameter<std::int64_t>("stale_timeout_ms", 100);
    const std::int64_t status_period_ms = declare_parameter<std::int64_t>("status_period_ms", 50);
    if (serial_path_.empty() || frame_id_.empty() || stale_timeout_ms <= 0 ||
      status_period_ms <= 0 ||
      baud_rate <= 0 || baud_rate > std::numeric_limits<int>::max() ||
      stale_timeout_ms > std::numeric_limits<int>::max() ||
      status_period_ms > std::numeric_limits<int>::max())
    {
      throw std::invalid_argument(
              "serial_path, frame_id, stale_timeout_ms, and status_period_ms must be valid");
    }
    baud_rate_ = static_cast<int>(baud_rate);
    // Validate prior to starting the worker so a bad baud never touches a port.
    (void)baud_to_termios(baud_rate_);
    stale_timeout_ = std::chrono::milliseconds(static_cast<int>(stale_timeout_ms));

    wrench_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/aubo/kwr75/raw_wrench", rclcpp::SensorDataQoS());
    health_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/aubo/kwr75/raw_wrench_valid", rclcpp::QoS(10));
    last_frame_time_ns_.store(0, std::memory_order_relaxed);
    worker_ = std::thread(&Kwr75SensorNode::serial_worker, this);
    health_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(status_period_ms)),
      std::bind(&Kwr75SensorNode::publish_health, this));
  }

  ~Kwr75SensorNode() override
  {
    running_.store(false, std::memory_order_release);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  bool is_fresh() const
  {
    if (!connected_.load(std::memory_order_acquire)) {
      return false;
    }
    const std::int64_t last_frame_time_ns = last_frame_time_ns_.load(std::memory_order_acquire);
    if (last_frame_time_ns == 0) {
      return false;
    }
    const auto last =
      std::chrono::steady_clock::time_point(std::chrono::nanoseconds(last_frame_time_ns));
    return std::chrono::steady_clock::now() - last <= stale_timeout_;
  }

  void publish_health()
  {
    std_msgs::msg::Bool message;
    message.data = is_fresh();
    health_publisher_->publish(message);
  }

  void publish_wrench(const Kwr75Wrench & wrench)
  {
    geometry_msgs::msg::WrenchStamped message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.wrench.force.x = wrench.fx_n;
    message.wrench.force.y = wrench.fy_n;
    message.wrench.force.z = wrench.fz_n;
    message.wrench.torque.x = wrench.mx_nm;
    message.wrench.torque.y = wrench.my_nm;
    message.wrench.torque.z = wrench.mz_nm;
    wrench_publisher_->publish(message);
  }

  void set_disconnected()
  {
    connected_.store(false, std::memory_order_release);
    last_frame_time_ns_.store(0, std::memory_order_release);
  }

  void serial_worker()
  {
    while (running_.load(std::memory_order_acquire)) {
      int file_descriptor = -1;
      try {
        file_descriptor = open_serial_8n1(serial_path_, baud_rate_);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Invalid serial configuration: %s", error.what());
        return;
      }
      if (file_descriptor < 0) {
        set_disconnected();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "KWR75D serial port unavailable: %s", serial_path_.c_str());
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }

      connected_.store(true, std::memory_order_release);
      Kwr75FrameParser parser;
      bool session_ok = true;
      if (start_stream_on_connect_ && !write_all(file_descriptor, start_continuous_command())) {
        RCLCPP_ERROR(get_logger(), "Could not send KWR75D continuous-stream command");
        session_ok = false;
      }

      std::array<std::uint8_t, 256U> bytes{};
      while (session_ok && running_.load(std::memory_order_acquire)) {
        pollfd descriptor{file_descriptor, POLLIN, 0};
        const int ready = poll(&descriptor, 1, static_cast<int>(kReadPollTimeout.count()));
        if (ready == 0) {
          continue;
        }
        if (ready < 0) {
          if (errno == EINTR) {
            continue;
          }
          session_ok = false;
          break;
        }
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
          session_ok = false;
          break;
        }
        if ((descriptor.revents & POLLIN) == 0) {
          continue;
        }
        const ssize_t received = read(file_descriptor, bytes.data(), bytes.size());
        if (received == 0) {
          session_ok = false;
          break;
        }
        if (received < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            continue;
          }
          session_ok = false;
          break;
        }
        for (const auto & wrench : parser.push(bytes.data(), static_cast<std::size_t>(received))) {
          last_frame_time_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count(),
            std::memory_order_release);
          publish_wrench(wrench);
        }
      }

      if (!running_.load(std::memory_order_acquire) && stop_stream_on_shutdown_) {
        (void)write_all(file_descriptor, stop_continuous_command());
      }
      close(file_descriptor);
      set_disconnected();
      if (running_.load(std::memory_order_acquire)) {
        RCLCPP_WARN(get_logger(), "KWR75D serial session ended; reconnecting");
        std::this_thread::sleep_for(kReconnectDelay);
      }
    }
  }

  std::string serial_path_;
  int baud_rate_{460800};
  std::string frame_id_;
  bool start_stream_on_connect_{true};
  bool stop_stream_on_shutdown_{true};
  std::chrono::milliseconds stale_timeout_{100};
  std::atomic<bool> running_{true};
  std::atomic<bool> connected_{false};
  std::atomic<std::int64_t> last_frame_time_ns_{0};
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_publisher_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::thread worker_;
};

}  // namespace aubo_kwr75_sensor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aubo_kwr75_sensor::Kwr75SensorNode>());
  rclcpp::shutdown();
  return 0;
}
