#ifndef CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainiksolvervel_pinv_givens.hpp"
#include "kdl/chainiksolvervel_wdls.hpp"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/bool.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_buffer.hpp"
#include "tf2_kdl/tf2_kdl.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp" // For hardware_interface::HW_IF_POSITION etc.

// 使用项目中的消息类型
#include "fairino_msgs/msg/pose_twist.hpp"

namespace cartesian_velocity_controller
{
class CartesianVelocityController : public controller_interface::ControllerInterface
{
public:
  // ROS 2控制器标准方法
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // ROS 2控制器生命周期方法 (使用现代API)
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

private:
  struct CommandState
  {
    geometry_msgs::msg::Twist twist;
    int64_t received_time_ns = 0;
    bool finite = false;
  };

  // 从URDF中构建KDL链
  bool build_kdl_chain();

  // Twist指令的回调函数
  void command_cart_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // KDL相关
  KDL::Chain kdl_chain_;
  KDL::JntArray q_pos_;           // 当前关节位置
  KDL::JntArray q_vel_;           // 当前关节速度
  KDL::JntArray q_vel_cmd_;       // 计算出的目标关节速度
  KDL::Twist cartesian_vel_cmd_;  // 目标笛卡尔速度

  // KDL求解器
  //正运动学——位置求解器
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  //速度求解器
  std::shared_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
  //逆运动学求解器，由笛卡尔速度求出关节速度
  std::shared_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;

  // ROS 2参数
  std::vector<std::string> joint_names_;
  std::string root_frame_;
  std::string tip_frame_;
  double publish_rate_ = 125.0;
  double command_timeout_s_ = 0.10;
  double ik_damping_ = 0.05;
  int max_ik_failures_ = 3;
  std::vector<double> joint_min_positions_;
  std::vector<double> joint_max_positions_;
  std::vector<double> joint_max_velocities_;
  std::vector<double> joint_max_accelerations_;

  // ROS 2通信
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cartesian_command_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<fairino_msgs::msg::PoseTwist>>
  ee_state_publisher_;
  realtime_tools::RealtimeBuffer<CommandState> command_buffer_;

  rclcpp::Time last_publish_time_;

  bool is_configured_ = false;
  int consecutive_ik_failures_ = 0;
  KDL::JntArray last_q_vel_cmd_;
};

}  // namespace cartesian_velocity_controller

#endif  // CARTESIAN_VELOCITY_CONTROLLER_HPP_
