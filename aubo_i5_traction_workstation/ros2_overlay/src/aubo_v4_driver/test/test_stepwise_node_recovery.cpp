// OFFLINE ONLY: compile the actual ROS node against the test-only SDK header.
// No vendor library is linked, no controller/serial connection can be opened.
#include <gtest/gtest.h>
#include <aubo_driver/serviceinterface.h>
#define main unused_production_entrypoint
#include "../src/aubo_v4_stepwise_node.cpp"
#undef main

using namespace std::chrono_literals;

TEST(StepwiseNodeRecovery, ThreeModesPauseRecoverAndRequireOperatorAfterHardFault)
{
  setenv("ROS_DOMAIN_ID", "177", 1);
  setenv("ROS_LOCALHOST_ONLY", "1", 1);
  setenv("AUBO_V4_HOST", "offline-sdk-double", 1);
  setenv("AUBO_V4_USERNAME", "offline-test", 1);
  setenv("AUBO_V4_PASSWORD", "offline-test", 1);
  const char * args[] = {"offline_node_recovery", "--ros-args", "-p", "connect_hardware:=true",
    "-p", "enable_motion_interface:=true", "-p", "enable_follow_drag:=true",
    "-p", "enable_follow_traction:=true", "-p", "follow_hold_enabled:=true",
    "-p", "experimental_tcp_can_drag:=true",
    "-p", "single_stop_when_quiescent:=true", "-p", "state_poll_interval_s:=0.02"};
  rclcpp::init(static_cast<int>(sizeof(args) / sizeof(args[0])), args);
  aubo_v4_test_sdk::reset();
  auto driver = std::make_shared<AuboV4StepwiseNode>();
  auto client = std::make_shared<rclcpp::Node>("offline_recovery_harness");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(client);
  auto mode_pub =
    client->create_publisher<fr_traction::msg::TractionCommand>("/traction/command", 1);
  auto velocity_pub = client->create_publisher<geometry_msgs::msg::Twist>(
    "/controller_manager/command_cart_vel", 1);
  auto switch_client = client->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");
  auto recover = client->create_client<std_srvs::srv::Trigger>("/aubo_v4_state_node/recover_fault");
  std::string phase;
  auto subscription = client->create_subscription<std_msgs::msg::String>(
    "/aubo_i5/stepwise_phase", rclcpp::QoS(1).transient_local(),
    [&phase](std_msgs::msg::String::SharedPtr message) {phase = message->data;});
  unsigned mode = fr_traction::msg::TractionCommand::TRACTION;
  const auto count = []() {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      return state.follow_move_count;
    };
  const auto pump = [&]() {
      aubo_robot_namespace::wayPoint_S point{};
      point.cartPos.position = {0.1, 0.2, 0.3};
      point.orientation.w = 1;
      aubo_v4_test_sdk::emit_waypoint(point);
      fr_traction::msg::TractionCommand command;
      command.header.stamp = client->now();
      command.header.frame_id = "base_link";
      command.mode = static_cast<uint8_t>(mode);
      mode_pub->publish(command);
      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = 0.002;
      velocity_pub->publish(velocity);
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    };
  const auto wait = [&](const auto & predicate) {
      const auto end = std::chrono::steady_clock::now() + 3s;
      while (!predicate() && std::chrono::steady_clock::now() < end) {pump();}
      return predicate();
    };
  EXPECT_TRUE(wait([&]() {return phase == "DISABLED" && switch_client->service_is_ready();}));
  for (const auto selected : {fr_traction::msg::TractionCommand::TRACTION,
      fr_traction::msg::TractionCommand::POSITIONING, fr_traction::msg::TractionCommand::DRAG})
  {
    mode = selected;
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      state.block_line_move = true;
      state.release_line_move = false;
    }
    const int before = count();
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = {"cartesian_velocity_controller"};
    auto started = switch_client->async_send_request(request);
    ASSERT_TRUE(wait([&]() {return started.wait_for(0ms) == std::future_status::ready;}));
    EXPECT_TRUE(started.get()->ok);
    EXPECT_TRUE(wait([&]() {return count() > before;}));
    bool false_fault = false;
    const auto delayed_reply_until = std::chrono::steady_clock::now() + 300ms;
    while (std::chrono::steady_clock::now() < delayed_reply_until) {
      pump();  // Actual ROS input/feedback continues while SDK double is busy.
      false_fault = false_fault || phase == "FAULT" || phase == "STOPPING";
    }
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      state.release_line_move = true;
      state.changed.notify_all();
      EXPECT_EQ(state.tcp_enter_count, 0);
      EXPECT_EQ(state.tcp_batch_count, 0);
    }
    EXPECT_FALSE(false_fault);  // All three modes tolerate the recorded delay.
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      state.line_move_result = 10023;
    }
    EXPECT_TRUE(wait([&]() {return phase == "STOPPING";}));
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      state.line_move_result = 0;
    }
    const int after_error = count();
    EXPECT_TRUE(wait([&]() {return count() > after_error && phase != "FAULT";}));
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      state.line_move_result = 10007;
    }
    EXPECT_TRUE(wait([&]() {return phase == "STOPPING";}));
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      state.line_move_result = 0;
    }
    auto stopping = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    stopping->stop_controllers = {"cartesian_velocity_controller"};
    auto cancelled = switch_client->async_send_request(stopping);
    ASSERT_TRUE(wait([&]() {return cancelled.wait_for(0ms) == std::future_status::ready;}));
    EXPECT_TRUE(cancelled.get()->ok);
    EXPECT_TRUE(wait([&]() {return phase == "DISABLED";}));
    const int cancelled_count = count();
    for (unsigned frame = 0; frame < 10; ++frame) {
      pump();
    }
    EXPECT_EQ(count(), cancelled_count);  // Operator stop revokes auto-resume.
    auto restarted = switch_client->async_send_request(request);
    ASSERT_TRUE(wait([&]() {return restarted.wait_for(0ms) == std::future_status::ready;}));
    EXPECT_TRUE(restarted.get()->ok);
    EXPECT_TRUE(wait([&]() {return count() > cancelled_count;}));
    {
      auto & state = aubo_v4_test_sdk::state();
      RobotEventCallback callback;
      void * context;
      {
        std::lock_guard<std::mutex> lock(state.mutex);
        callback = state.event_callback;
        context = state.event_context;
      }
      aubo_robot_namespace::RobotEventInfo event;
      event.eventType = aubo_robot_namespace::RobotEventJointOutRange;
      event.eventCode = 21006;
      event.eventContent = "offline joint limit regression";
      callback(&event, context);
    }
    EXPECT_TRUE(wait([&]() {return phase == "FAULT";}));
    bool recovered = false;
    for (unsigned attempt = 0; attempt < 30 && !recovered; ++attempt) {
      auto future =
        recover->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      ASSERT_TRUE(wait([&]() {return future.wait_for(0ms) == std::future_status::ready;}));
      recovered = future.get()->success;
      if (!recovered) {
        for (unsigned frame = 0; frame < 10; ++frame) {
          pump();
        }
      }
    }
    EXPECT_TRUE(recovered);
    EXPECT_TRUE(wait([&]() {return phase == "DISABLED";}));
    const int stopped_count = count();
    for (unsigned frame = 0; frame < 10; ++frame) {
      pump();
    }
    EXPECT_EQ(count(), stopped_count);  // Fresh velocities alone cannot restart.
  }
  executor.remove_node(client);
  executor.remove_node(driver);
  client.reset();
  driver.reset();
  rclcpp::shutdown();
}
