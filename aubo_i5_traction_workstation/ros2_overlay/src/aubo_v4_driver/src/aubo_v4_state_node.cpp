#include <aubo_driver/AuboRobotMetaType.h>
#include <aubo_driver/serviceinterface.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <fr_traction/msg/end_effector_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <array>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include "aubo_v4_driver/state_validation.hpp"

namespace
{
constexpr int kDefaultPort = 8899;
std::optional<std::string> env(const char * name)
{
  const char * v = std::getenv(name); if (v == nullptr || v[0] == '\0') {
    return std::nullopt;
  }
  return std::string(v);
}
std::optional<int> port()
{
  const auto v = env("AUBO_V4_PORT"); if (!v) {
    return kDefaultPort;
  }
  try {
    const int p = std::stoi(*v); if (p > 0 && p <= 65535) {
      return p;
    }
  } catch (const std::exception &) {
  }
  return std::nullopt;
}
template<typename T> diagnostic_msgs::msg::KeyValue kv(const std::string & key, const T & value)
{
  diagnostic_msgs::msg::KeyValue out; out.key = key; std::ostringstream s; s << value;
  out.value = s.str(); return out;
}
}  // namespace

class AuboV4StateNode : public rclcpp::Node
{
public:
  AuboV4StateNode()
  : Node("aubo_v4_state_node"),
    joint_names_(declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint",
          "wrist2_joint", "wrist3_joint"})),
    pose_frame_id_(declare_parameter<std::string>(
        "pose_frame_id",
        "aubo_base"))
  {
    enable_motion_interface_ = declare_parameter<bool>("enable_motion_interface", false);
    const auto period_ms = declare_parameter<std::int64_t>("state_period_ms", 100);
    if (joint_names_.size() != aubo_v4_driver::kArmDof || pose_frame_id_.empty() ||
      period_ms <= 0)
    {
      error_ =
        "invalid node parameters: six joint names, non-empty pose_frame_id, and positive state_period_ms are required";
    }
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    ee_pub_ = create_publisher<fr_traction::msg::EndEffectorState>(
      "/controller_manager/ee_state",
      10);
    health_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/controller_manager/healthy", rclcpp::QoS(
        1).transient_local());
    identity_pub_ =
      create_publisher<std_msgs::msg::String>(
      "/aubo_i5/driver_identity", rclcpp::QoS(
        1).transient_local());
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(1).transient_local());
    stop_service_ =
      create_service<std_srvs::srv::Trigger>(
      "~/hardware_stop",
      std::bind(&AuboV4StateNode::stop, this, std::placeholders::_1, std::placeholders::_2));
    if (enable_motion_interface_) {
      command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "/controller_manager/command_cart_vel", rclcpp::QoS(10).reliable(),
        std::bind(&AuboV4StateNode::on_velocity, this, std::placeholders::_1));
      switch_service_ = create_service<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller",
        std::bind(
          &AuboV4StateNode::switch_controller, this,
          std::placeholders::_1, std::placeholders::_2));
    }
    if (error_.empty()) {connect_once();}
    publish_identity(); publish_status();
    identity_timer_ =
      create_wall_timer(
      std::chrono::seconds(1),
      [this]() {publish_identity(); publish_status();});
    if (error_.empty()) {
      timer_ =
        create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&AuboV4StateNode::poll, this));
      if (enable_motion_interface_) {
        motion_timer_ = create_wall_timer(
          std::chrono::milliseconds(5), std::bind(&AuboV4StateNode::motion_tick, this));
        // A continuously-ready 5 ms timer can starve state polling and service
        // callbacks in the single-threaded executor even while no stream is active.
        // Arm it only for an accepted TCP2CAN session.
        motion_timer_->cancel();
      }
    }
  }
  ~AuboV4StateNode() override
  {
    if (stream_active_) {
      const int ignored = service_.robotServiceLeaveTcp2CanbusMode(); (void)ignored;
    }
    if (connected_) {
      const int ignored = service_.robotServiceLogout(); (void)ignored;
    }
  }

private:
  void stop_stream(const std::string & failure = {})
  {
    if (!failure.empty()) {
      RCLCPP_ERROR(get_logger(), "V4 stream stopped: %s", failure.c_str());
    }
    if (motion_timer_) {motion_timer_->cancel();}
    if (!failure.empty() && connected_) {
      const int ignored = service_.robotMoveFastStop(); (void)ignored;
      stop_requested_ = true;
    }
    if (stream_active_) {
      const int result = service_.robotServiceLeaveTcp2CanbusMode();
      stream_active_ = false;
      if (result != aubo_robot_namespace::ErrnoSucc && failure.empty()) {
        fail_closed_ = true;
        error_ = "V4 stream exit failed (SDK result " + std::to_string(result) + ")";
        RCLCPP_ERROR(get_logger(), "%s", error_.c_str());
      }
    }
    commanded_velocity_ = {};
    if (!failure.empty()) {fail_closed_ = true; error_ = failure;}
    publish_status();
  }
  void switch_controller(
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response)
  {
    constexpr const char * kName = "cartesian_velocity_controller";
    const auto valid_names = [kName](const std::vector<std::string> & names) {
        return std::all_of(
          names.begin(), names.end(), [kName](const std::string & n) {
            return n == kName;
          });
      };
    if (!valid_names(request->stop_controllers) ||
      !valid_names(request->start_controllers))
    {
      response->ok = false; return;
    }
    if (!request->stop_controllers.empty()) {stop_stream();}
    if (request->start_controllers.empty()) {response->ok = true; return;}
    if (!healthy() || stream_active_) {response->ok = false; return;}
    aubo_robot_namespace::wayPoint_S waypoint{};
    if (service_.robotServiceGetCurrentWaypointInfo(waypoint) != aubo_robot_namespace::ErrnoSucc ||
      !aubo_v4_driver::valid_pose_sample(pose(waypoint), nullptr))
    {
      response->ok = false; return;
    }
    const int result = service_.robotServiceEnterTcp2CanbusMode();
    if (result != aubo_robot_namespace::ErrnoSucc) {
      fail_closed_ = true;
      error_ = "V4 stream entry failed (SDK result " + std::to_string(result) + ")";
      publish_status(); response->ok = false; return;
    }
    target_waypoint_ = waypoint;
    commanded_velocity_ = {};
    last_velocity_at_ = std::chrono::steady_clock::now();
    last_motion_tick_at_ = last_velocity_at_;
    stream_active_ = true;
    if (motion_timer_) {motion_timer_->reset();}
    response->ok = true;
    publish_status();
  }
  void on_velocity(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const double x = message->linear.x, y = message->linear.y, z = message->linear.z;
    const double speed = std::sqrt(x * x + y * y + z * z);
    if (!std::isfinite(speed) || speed > 0.051 ||
      !std::isfinite(message->angular.x) || !std::isfinite(message->angular.y) ||
      !std::isfinite(message->angular.z) ||
      std::abs(message->angular.x) > 1e-9 ||
      std::abs(message->angular.y) > 1e-9 || std::abs(message->angular.z) > 1e-9)
    {
      if (stream_active_) {stop_stream("invalid V4 Cartesian velocity command");}
      return;
    }
    commanded_velocity_ = {x, y, z};
    last_velocity_at_ = std::chrono::steady_clock::now();
  }
  void motion_tick()
  {
    if (!stream_active_) {return;}
    if (!healthy()) {stop_stream("V4 feedback became unhealthy during streaming"); return;}
    const auto current = std::chrono::steady_clock::now();
    const double age = std::chrono::duration<double>(current - last_velocity_at_).count();
    if (age > 0.25) {stop_stream("V4 velocity command watchdog expired"); return;}
    const double dt = std::chrono::duration<double>(current - last_motion_tick_at_).count();
    last_motion_tick_at_ = current;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.03) {
      stop_stream("V4 joint stream timing gap exceeded 30 ms"); return;
    }
    double joints[aubo_robot_namespace::ARM_DOF]{};
    const double speed = std::sqrt(
      commanded_velocity_[0] * commanded_velocity_[0] +
      commanded_velocity_[1] * commanded_velocity_[1] +
      commanded_velocity_[2] * commanded_velocity_[2]);
    if (speed > 1e-9) {
      auto position = target_waypoint_.cartPos.position;
      position.x += commanded_velocity_[0] * dt;
      position.y += commanded_velocity_[1] * dt;
      position.z += commanded_velocity_[2] * dt;
      if (previous_) {
        const double dx = position.x - previous_->position[0];
        const double dy = position.y - previous_->position[1];
        const double dz = position.z - previous_->position[2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.02) {
          stop_stream("V4 commanded path diverged from measured pose"); return;
        }
      }
      aubo_robot_namespace::wayPoint_S solved{};
      const int ik = service_.robotServiceRobotIk(
        target_waypoint_.jointpos, position, target_waypoint_.orientation, solved);
      if (ik != aubo_robot_namespace::ErrnoSucc) {
        stop_stream("V4 IK failed (SDK result " + std::to_string(ik) + ")"); return;
      }
      for (std::size_t i = 0; i < aubo_v4_driver::kArmDof; ++i) {
        if (!std::isfinite(solved.jointpos[i]) ||
          std::abs(solved.jointpos[i] - target_waypoint_.jointpos[i]) > 0.02)
        {
          stop_stream("V4 IK produced an invalid joint step"); return;
        }
      }
      target_waypoint_ = solved;
    }
    std::copy_n(target_waypoint_.jointpos, aubo_v4_driver::kArmDof, joints);
    const int send = service_.robotServiceSetRobotPosData2Canbus(joints);
    if (send != aubo_robot_namespace::ErrnoSucc) {
      stop_stream("V4 joint stream failed (SDK result " + std::to_string(send) + ")");
    }
  }
  void connect_once()
  {
    const auto host = env("AUBO_V4_HOST"); const auto user = env("AUBO_V4_USERNAME");
    const auto password = env("AUBO_V4_PASSWORD"); const auto controller_port = port();
    if (!host || !user || !password || !controller_port) {
      error_ =
        "V4 login is disabled: AUBO_V4_HOST, AUBO_V4_USERNAME, AUBO_V4_PASSWORD, and a valid AUBO_V4_PORT are required";
      return;
    }
    const int result = service_.robotServiceLogin(
      host->c_str(), *controller_port,
      user->c_str(), password->c_str());
    if (result != aubo_robot_namespace::ErrnoSucc) {
      error_ = "V4 login failed (SDK result " + std::to_string(result) + ")"; return;
    }
    connected_ = true;
    RCLCPP_INFO(
      get_logger(),
      "AUBOPE V4 state and explicit Cartesian velocity interface connected");
  }
  static aubo_v4_driver::PoseSample pose(const aubo_robot_namespace::wayPoint_S & w)
  {
    aubo_v4_driver::PoseSample s;
    s.position = {w.cartPos.position.x, w.cartPos.position.y, w.cartPos.position.z};
    s.orientation = {w.orientation.x, w.orientation.y, w.orientation.z, w.orientation.w};
    for (std::size_t i = 0; i < aubo_v4_driver::kArmDof; ++i) {
      s.joints[i] = w.jointpos[i];
    }
    return s;
  }
  static aubo_v4_driver::DiagnosisSample diagnosis(const aubo_robot_namespace::RobotDiagnosis & d)
  {
    aubo_v4_driver::DiagnosisSample s; s.arm_power_status = d.armPowerStatus;
    s.arm_canbus_status = d.armCanbusStatus; s.remote_halt = d.remoteHalt;
    s.soft_emergency = d.softEmergency; s.remote_emergency = d.remoteEmergency;
    s.robot_collision = d.robotCollision; s.encoder_error_status = d.encoderErrorStatus;
    s.joint_collision_detect = d.jointCollisionDetect;
    s.encoder_lines_error = d.encoderLinesError; s.joint_error_status = d.jointErrorStatus;
    s.singularity_overspeed_alarm = d.singularityOverSpeedAlarm;
    s.robot_current_alarm = d.robotCurrentAlarm; s.tool_io_error = d.toolIoError; return s;
  }
  void poll()
  {
    if (!connected_ || fail_closed_ || stop_requested_) {return;}
    aubo_robot_namespace::wayPoint_S waypoint{};
    aubo_robot_namespace::RobotDiagnosis raw_diagnosis{};
    const int waypoint_result = service_.robotServiceGetCurrentWaypointInfo(waypoint);
    const int diagnosis_result = service_.robotServiceGetRobotDiagnosisInfo(raw_diagnosis);
    if (waypoint_result != aubo_robot_namespace::ErrnoSucc ||
      diagnosis_result != aubo_robot_namespace::ErrnoSucc)
    {
      fail_closed_ = true; error_ = "V4 state read failed (waypoint=" + std::to_string(
        waypoint_result) + ", diagnosis=" + std::to_string(diagnosis_result) +
        "); restart required; no automatic recovery"; publish_status(); return;
    }
    const auto sample = pose(waypoint); std::string reason;
    if (!aubo_v4_driver::valid_pose_sample(sample, &reason)) {
      fail_closed_ = true; error_ = reason + "; restart required; no automatic recovery";
      publish_status(); return;
    }
    diagnosis_healthy_ = aubo_v4_driver::diagnosis_is_healthy(
      diagnosis(
        raw_diagnosis), &diagnosis_reason_); last_read_ = true;
    last_read_at_ = std::chrono::steady_clock::now();
    publish_state(sample);
    publish_status();
  }
  void publish_state(const aubo_v4_driver::PoseSample & s)
  {
    const auto stamp = now(); sensor_msgs::msg::JointState joints; joints.header.stamp = stamp;
    joints.name = joint_names_; joints.position.assign(s.joints.begin(), s.joints.end());
    if (previous_) {
      const double dt = (stamp - previous_stamp_).seconds();
      if (dt > 0.0 && dt <= 1.0) {
        joints.velocity.reserve(aubo_v4_driver::kArmDof);
        for (std::size_t i = 0; i < aubo_v4_driver::kArmDof; ++i) {
          joints.velocity.push_back((s.joints[i] - previous_->joints[i]) / dt);
        }
      }
    }
    joint_pub_->publish(joints);
    fr_traction::msg::EndEffectorState ee; ee.header.stamp = stamp;
    ee.header.frame_id = pose_frame_id_; ee.pose.position.x = s.position[0];
    ee.pose.position.y = s.position[1]; ee.pose.position.z = s.position[2];
    ee.pose.orientation.x = s.orientation[0]; ee.pose.orientation.y = s.orientation[1];
    ee.pose.orientation.z = s.orientation[2]; ee.pose.orientation.w = s.orientation[3];
    // V4 waypoint polling has no verified twist source. Estimate only linear velocity from two valid samples; angular twist stays zero.
    if (previous_) {
      const double dt = (stamp - previous_stamp_).seconds(); if (dt > 0.0 && dt <= 1.0) {
        ee.twist.linear.x = (s.position[0] - previous_->position[0]) / dt;
        ee.twist.linear.y = (s.position[1] - previous_->position[1]) / dt;
        ee.twist.linear.z = (s.position[2] - previous_->position[2]) / dt;
      }
    }
    ee_pub_->publish(ee); previous_ = s; previous_stamp_ = stamp;
  }
  bool healthy() const
  {
    const double feedback_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_read_at_).count();
    return connected_ && !fail_closed_ && !stop_requested_ && last_read_ && diagnosis_healthy_ &&
           feedback_age <= 0.25;
  }
  void publish_identity()
  {
    std_msgs::msg::String identity; identity.data = "AUBO_I5_V4"; identity_pub_->publish(identity);
  }
  std::string reason() const
  {
    if (!error_.empty()) {
      return error_;
    }
    if (stop_requested_) {
      return
        "hardware stop requested; operator review and node restart are required before health can become true";
    }
    if (!diagnosis_reason_.empty()) {
      if (last_read_ && diagnosis_healthy_ &&
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_read_at_).count() > 0.25)
      {
        return "V4 state feedback is stale";
      }
      return diagnosis_reason_;
    }
    return "no verified V4 state sample has been read";
  }
  void publish_status()
  {
    std_msgs::msg::Bool health; health.data = healthy(); health_pub_->publish(health);
    diagnostic_msgs::msg::DiagnosticStatus status; status.name = "aubo_v4_driver/state";
    status.hardware_id = "AUBOPE V4";
    status.level =
      health.data ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::
      DiagnosticStatus::ERROR; status.message = health.data ? diagnosis_reason_ : reason();
    status.values.push_back(kv("connected", connected_));
    status.values.push_back(kv("state_read_succeeded", last_read_));
    status.values.push_back(kv("diagnosis_healthy", diagnosis_healthy_)); status.values.push_back(
      kv(
        "stop_requested",
        stop_requested_)); status.values.push_back(kv("fail_closed", fail_closed_));
    status.values.push_back(kv("motion_interface_enabled", enable_motion_interface_));
    status.values.push_back(kv("motion_stream_active", stream_active_));
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = now(); array.status.push_back(
      std::move(
        status)); diag_pub_->publish(array);
  }
  void stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!connected_) {
      response->success = false;
      response->message = "hardware stop rejected: no authenticated V4 connection";
      publish_status(); return;
    }
    const int result = service_.robotMoveFastStop();
    stream_active_ = false;
    commanded_velocity_ = {};
    if (result != aubo_robot_namespace::ErrnoSucc) {
      fail_closed_ = true; error_ = "V4 fast-stop request failed (SDK result " + std::to_string(
        result) + ")"; response->success = false; response->message = error_; publish_status();
      return;
    }
    stop_requested_ = true; response->success = true;
    response->message =
      "V4 fast-stop request returned success; health remains false until manual operator review and node restart";
    publish_status();
  }
  ServiceInterface service_; std::vector<std::string> joint_names_; std::string pose_frame_id_;
  bool connected_{false}; bool fail_closed_{false}; bool stop_requested_{false};
  bool enable_motion_interface_{false};
  bool last_read_{false}; bool diagnosis_healthy_{false}; std::string error_;
  std::chrono::steady_clock::time_point last_read_at_{};
  std::string diagnosis_reason_; std::optional<aubo_v4_driver::PoseSample> previous_;
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<fr_traction::msg::EndEffectorState>::SharedPtr ee_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr identity_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_; rclcpp::TimerBase::SharedPtr identity_timer_;
  rclcpp::TimerBase::SharedPtr motion_timer_;
  bool stream_active_{false};
  aubo_robot_namespace::wayPoint_S target_waypoint_{};
  std::array<double, 3> commanded_velocity_{};
  std::chrono::steady_clock::time_point last_velocity_at_{};
  std::chrono::steady_clock::time_point last_motion_tick_at_{};
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<AuboV4StateNode>()); rclcpp::shutdown();
  return 0;
}
