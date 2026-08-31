#include "fr_controllers/cartesian_velocity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cartesian_velocity_controller
{

controller_interface::CallbackReturn CartesianVelocityController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  auto_declare<std::string>("root_frame", "");
  auto_declare<std::string>("tip_frame", "");
  auto_declare<double>("publish_rate", 125.0);
  auto_declare<double>("command_timeout_s", 0.10);
  auto_declare<double>("ik_damping", 0.05);
  auto_declare<int>("max_ik_failures", 3);
  auto_declare<std::vector<double>>("joint_min_positions", std::vector<double>());
  auto_declare<std::vector<double>>("joint_max_positions", std::vector<double>());
  auto_declare<std::vector<double>>("joint_max_velocities", std::vector<double>());
  auto_declare<std::vector<double>>("joint_max_accelerations", std::vector<double>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  root_frame_ = get_node()->get_parameter("root_frame").as_string();
  tip_frame_ = get_node()->get_parameter("tip_frame").as_string();
  if (joint_names_.empty() || root_frame_.empty() || tip_frame_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "joints, root_frame and tip_frame are required.");
    return controller_interface::CallbackReturn::ERROR;
  }

  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  command_timeout_s_ = get_node()->get_parameter("command_timeout_s").as_double();
  ik_damping_ = get_node()->get_parameter("ik_damping").as_double();
  max_ik_failures_ = get_node()->get_parameter("max_ik_failures").as_int();
  joint_min_positions_ = get_node()->get_parameter("joint_min_positions").as_double_array();
  joint_max_positions_ = get_node()->get_parameter("joint_max_positions").as_double_array();
  joint_max_velocities_ = get_node()->get_parameter("joint_max_velocities").as_double_array();
  joint_max_accelerations_ = get_node()->get_parameter("joint_max_accelerations").as_double_array();

  const auto expected = joint_names_.size();
  if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0 ||
    !std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0 ||
    !std::isfinite(ik_damping_) || ik_damping_ <= 0.0 || max_ik_failures_ < 1 ||
    joint_min_positions_.size() != expected || joint_max_positions_.size() != expected ||
    joint_max_velocities_.size() != expected || joint_max_accelerations_.size() != expected)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid Cartesian controller or joint limit parameters.");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < expected; ++i) {
    if (!std::isfinite(joint_min_positions_[i]) || !std::isfinite(joint_max_positions_[i]) ||
      joint_min_positions_[i] >= joint_max_positions_[i] ||
      !std::isfinite(joint_max_velocities_[i]) || joint_max_velocities_[i] <= 0.0 ||
      !std::isfinite(joint_max_accelerations_[i]) || joint_max_accelerations_[i] <= 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid limits for joint %zu.", i);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if (!build_kdl_chain()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  q_pos_.resize(kdl_chain_.getNrOfJoints());
  q_vel_.resize(kdl_chain_.getNrOfJoints());
  q_vel_cmd_.resize(kdl_chain_.getNrOfJoints());
  last_q_vel_cmd_.resize(kdl_chain_.getNrOfJoints());
  KDL::SetToZero(q_vel_cmd_);
  KDL::SetToZero(last_q_vel_cmd_);

  fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  fk_vel_solver_ = std::make_shared<KDL::ChainFkSolverVel_recursive>(kdl_chain_);
  ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_wdls>(kdl_chain_);
  ik_vel_solver_->setLambda(ik_damping_);

  cartesian_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/command_cart_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {command_cart_vel_callback(msg);});
  health_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    "~/healthy", rclcpp::QoS(1).transient_local().reliable());
  ee_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<fairino_msgs::msg::PoseTwist>>(
    get_node()->create_publisher<fairino_msgs::msg::PoseTwist>(
      "~/ee_state", rclcpp::SystemDefaultsQoS()));
  last_publish_time_ = get_node()->get_clock()->now();
  command_buffer_.initRT(CommandState{});
  consecutive_ik_failures_ = 0;
  is_configured_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Configured with damped WDLS IK and Cartesian watchdog.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.size() < joint_names_.size() * 2 ||
    command_interfaces_.size() < joint_names_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate without complete joint interfaces.");
    return controller_interface::CallbackReturn::ERROR;
  }
  // The official teach pendant may have moved the robot while this controller
  // was inactive. Start from the actual feedback pose, never from a stale
  // command left before manual setup.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double position = state_interfaces_[i * 2].get_value();
    if (!std::isfinite(position)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Non-finite joint position at activation.");
      return controller_interface::CallbackReturn::ERROR;
    }
    command_interfaces_[i].set_value(position);
  }
  KDL::SetToZero(q_vel_cmd_);
  KDL::SetToZero(last_q_vel_cmd_);
  cartesian_vel_cmd_ = KDL::Twist::Zero();
  consecutive_ik_failures_ = 0;
  command_buffer_.writeFromNonRT(CommandState{});
  if (health_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    health_publisher_->publish(msg);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  KDL::SetToZero(q_vel_cmd_);
  KDL::SetToZero(last_q_vel_cmd_);
  cartesian_vel_cmd_ = KDL::Twist::Zero();
  if (health_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = false;
    health_publisher_->publish(msg);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::
command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::
state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

controller_interface::return_type CartesianVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!is_configured_ || state_interfaces_.size() < joint_names_.size() * 2 ||
    command_interfaces_.size() < joint_names_.size())
  {
    return controller_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.25) {
    KDL::SetToZero(q_vel_cmd_);
    KDL::SetToZero(last_q_vel_cmd_);
    return controller_interface::return_type::ERROR;
  }

  bool fatal_error = false;
  const auto * command_state = command_buffer_.readFromRT();
  const int64_t now_ns = time.nanoseconds();
  const bool time_is_monotonic = command_state->received_time_ns > 0 &&
    now_ns >= command_state->received_time_ns;
  const double command_age_s = time_is_monotonic ?
    static_cast<double>(now_ns - command_state->received_time_ns) * 1e-9 :
    std::numeric_limits<double>::infinity();
  const bool command_fresh = command_state->finite && time_is_monotonic &&
    command_age_s <= command_timeout_s_;
  if (command_state->received_time_ns > 0 && !command_state->finite) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Rejected a non-finite Cartesian velocity command; output is zero.");
    fatal_error = true;
  } else if (!command_fresh) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Cartesian velocity command is stale or time moved backwards; output is zero.");
  }

  bool state_finite = true;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_pos_(i) = state_interfaces_[i * 2].get_value();
    q_vel_(i) = state_interfaces_[i * 2 + 1].get_value();
    state_finite = state_finite && std::isfinite(q_pos_(i)) && std::isfinite(q_vel_(i));
  }
  if (!state_finite) {
    RCLCPP_ERROR(get_node()->get_logger(), "Non-finite joint state; output is zero.");
    fatal_error = true;
  }

  if (!fatal_error && command_fresh) {
    cartesian_vel_cmd_.vel.x(command_state->twist.linear.x);
    cartesian_vel_cmd_.vel.y(command_state->twist.linear.y);
    cartesian_vel_cmd_.vel.z(command_state->twist.linear.z);
    cartesian_vel_cmd_.rot.x(command_state->twist.angular.x);
    cartesian_vel_cmd_.rot.y(command_state->twist.angular.y);
    cartesian_vel_cmd_.rot.z(command_state->twist.angular.z);

    const int ik_result = ik_vel_solver_->CartToJnt(q_pos_, cartesian_vel_cmd_, q_vel_cmd_);
    bool qdot_finite = true;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      qdot_finite = qdot_finite && std::isfinite(q_vel_cmd_(i));
    }
    if (ik_result != KDL::ChainIkSolverVel::E_NOERROR || !qdot_finite) {
      KDL::SetToZero(q_vel_cmd_);
      KDL::SetToZero(last_q_vel_cmd_);
      ++consecutive_ik_failures_;
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Cartesian IK failed (result=%d, consecutive=%d); output is zero.",
        ik_result, consecutive_ik_failures_);
      if (consecutive_ik_failures_ >= max_ik_failures_) {
        fatal_error = true;
      }
    } else {
      consecutive_ik_failures_ = 0;
      bool candidate_valid = true;
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        double bounded = std::clamp(
          q_vel_cmd_(i), -joint_max_velocities_[i], joint_max_velocities_[i]);
        const double max_delta = joint_max_accelerations_[i] * dt;
        bounded = last_q_vel_cmd_(i) + std::clamp(
          bounded - last_q_vel_cmd_(i), -max_delta, max_delta);
        q_vel_cmd_(i) = bounded;
        const double current_command = command_interfaces_[i].get_value();
        const double next_position = current_command + bounded * dt;
        if (!std::isfinite(current_command) || !std::isfinite(next_position) ||
          current_command < joint_min_positions_[i] || current_command > joint_max_positions_[i] ||
          next_position<joint_min_positions_[i] || next_position> joint_max_positions_[i])
        {
          candidate_valid = false;
        }
      }
      if (!candidate_valid) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Joint position boundary would be crossed; rejecting this command cycle.");
        KDL::SetToZero(q_vel_cmd_);
        KDL::SetToZero(last_q_vel_cmd_);
      } else {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          command_interfaces_[i].set_value(
            command_interfaces_[i].get_value() + q_vel_cmd_(i) * dt);
          last_q_vel_cmd_(i) = q_vel_cmd_(i);
        }
      }
    }
  } else {
    KDL::SetToZero(q_vel_cmd_);
    KDL::SetToZero(last_q_vel_cmd_);
  }

  if (state_finite && time > last_publish_time_ + rclcpp::Duration::from_seconds(
      1.0 / publish_rate_))
  {
    KDL::Frame current_pose;
    KDL::FrameVel current_vel_kdl;
    if (fk_pos_solver_->JntToCart(q_pos_, current_pose) < 0 ||
      fk_vel_solver_->JntToCart(KDL::JntArrayVel(q_pos_, q_vel_), current_vel_kdl) < 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to calculate end-effector state.");
      fatal_error = true;
    } else if (ee_state_publisher_ && ee_state_publisher_->trylock()) {
      auto & pub = ee_state_publisher_->msg_;
      pub.header.stamp = time;
      pub.header.frame_id = root_frame_;
      pub.pose = tf2::toMsg(current_pose);
      const KDL::Twist & twist_kdl = current_vel_kdl.GetTwist();
      pub.twist.linear.x = twist_kdl.vel.x();
      pub.twist.linear.y = twist_kdl.vel.y();
      pub.twist.linear.z = twist_kdl.vel.z();
      pub.twist.angular.x = twist_kdl.rot.x();
      pub.twist.angular.y = twist_kdl.rot.y();
      pub.twist.angular.z = twist_kdl.rot.z();
      ee_state_publisher_->unlockAndPublish();
      last_publish_time_ = time;
    }
  }

  if (health_publisher_) {
    std_msgs::msg::Bool msg;
    msg.data = !fatal_error;
    health_publisher_->publish(msg);
  }
  return fatal_error ? controller_interface::return_type::ERROR : controller_interface::return_type
         ::OK;
}

bool CartesianVelocityController::build_kdl_chain()
{
  const auto urdf_string = get_node()->get_parameter("robot_description").as_string();
  if (urdf_string.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "URDF is empty, could not build KDL chain.");
    return false;
  }
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree) ||
    !kdl_tree.getChain(root_frame_, tip_frame_, kdl_chain_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to construct KDL chain from %s to %s.",
      root_frame_.c_str(), tip_frame_.c_str());
    return false;
  }
  if (kdl_chain_.getNrOfJoints() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "KDL chain joint count does not match configuration.");
    return false;
  }
  return true;
}

void CartesianVelocityController::command_cart_vel_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  CommandState state;
  state.twist = *msg;
  state.received_time_ns = get_node()->get_clock()->now().nanoseconds();
  state.finite = std::isfinite(msg->linear.x) && std::isfinite(msg->linear.y) &&
    std::isfinite(msg->linear.z) && std::isfinite(msg->angular.x) &&
    std::isfinite(msg->angular.y) && std::isfinite(msg->angular.z);
  command_buffer_.writeFromNonRT(state);
}

}  // namespace cartesian_velocity_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cartesian_velocity_controller::CartesianVelocityController,
  controller_interface::ControllerInterface)
