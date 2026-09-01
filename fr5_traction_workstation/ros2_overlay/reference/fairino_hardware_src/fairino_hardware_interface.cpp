#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "fairino_hardware/finite_checks.hpp"

#include <arpa/inet.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace
{

bool is_valid_ipv4(const std::string & address)
{
  in_addr parsed{};
  return !address.empty() && inet_pton(AF_INET, address.c_str(), &parsed) == 1;
}

bool parse_bool_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  bool default_value)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return default_value;
  }
  if (it->second == "true" || it->second == "True" || it->second == "TRUE" ||
    it->second == "1") {
    return true;
  }
  if (it->second == "false" || it->second == "False" || it->second == "FALSE" ||
    it->second == "0") {
    return false;
  }
  throw std::invalid_argument("invalid boolean value for " + name + ": " + it->second);
}

}  // namespace

namespace fairino_hardware
{
//首先检查传入的参数是不是合法
hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & sysinfo)
{
  if (hardware_interface::SystemInterface::on_init(sysinfo) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  info_ = sysinfo;  //info_是父类中定义的变量

  const auto ip_it = info_.hardware_parameters.find("robot_ip");
  if (ip_it != info_.hardware_parameters.end()) {
    _controller_ip = ip_it->second;
  }
  if (!is_valid_ipv4(_controller_ip)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Invalid robot_ip '%s'. Expected an IPv4 address.", _controller_ip.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    _zero_sensor_on_activate = parse_bool_parameter(
      info_.hardware_parameters, "zero_sensor_on_activate", true);
  } catch (const std::invalid_argument & error) {
    RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "%s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {

    //指令部分
    if (joint.command_interfaces.size() != 1) {    //开放servoJ
      RCLCPP_FATAL(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
    //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
    //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
    //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    //关节状态部分
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "FairinoHardwareInterface"),
        "Joint '%s' have %s state interface as first state interface. '%s' expected.",
        joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "FairinoHardwareInterface"),
        "Joint '%s' have %s state interface as second state interface. '%s' expected.",
        joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
    //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
    //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
    //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

  }
  return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init


std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
  }
// --- 关键修改：导出六轴力传感器的状态接口 ---
  // 我们需要遍历在URDF中定义的<sensor>标签
  for (const auto & sensor : info_.sensors) {
    // 假设传感器的名称在URDF中为 "tcp_force_torque_sensor"
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        sensor.name, "force.x", &_ft_sensor_state[0]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        sensor.name, "force.y", &_ft_sensor_state[1]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        sensor.name, "force.z", &_ft_sensor_state[2]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        sensor.name, "torque.x", &_ft_sensor_state[3]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        sensor.name, "torque.y", &_ft_sensor_state[4]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        sensor.name, "torque.z", &_ft_sensor_state[5]));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}


//ros2_control走生命周期管理，
hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  using namespace std::chrono_literals;
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
  //做变量的初始化工作
  _ptr_robot = std::make_unique<FRRobot>();  //创建机器人实例
  _consecutive_ft_failures = 0;
  _servoj_enabled = false;
  _servo_session_active = false;
  for (int i = 0; i < 6; i++) {//初始化变量
    _jnt_position_command[i] = 0;
    _jnt_velocity_command[i] = 0;
    _jnt_torque_command[i] = 0;
    _jnt_position_state[i] = 0;
    _jnt_velocity_state[i] = 0;
    _jnt_torque_state[i] = 0;
    _ft_sensor_state[i] = 0;
  }
  _control_mode = 0;  //默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
  errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());  //建立xmlrpc连接
  rclcpp::sleep_for(200ms);  //等待一段时间让控制器的rpc连接建立完毕
  if (returncode != 0) {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接失败！请检查端口时候被占用");
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接成功！");
  }
  //做第一步的工作，读取当前状态数据
  JointPos jntpos;
  returncode = _ptr_robot->GetActualJointPosDegree(0, &jntpos);
  // The SDK returns RCS data in the selected reference frame. Use base_link
  // explicitly so the broadcaster frame_id has the same meaning.
  DescPose zero_pose{};
  errno_t rcs_returncode = _ptr_robot->FT_SetRCS(1, zero_pose);
  if (rcs_returncode != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "FT_SetRCS(base_link) failed with code %d.", rcs_returncode);
    _ptr_robot->CloseRPC();
    _ptr_robot.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("FairinoHardwareInterface"),
    "FT reference coordinate system set to base_link (FT_SetRCS ref=1).");

  if (_zero_sensor_on_activate) {
    errno_t zero_returncode = _ptr_robot->FT_SetZero(1);
    if (zero_returncode != 0) {
      // Some KWR75D/controller firmware combinations reject FT_SetZero while
      // the robot is in the current teach-pendant state. Keep the live sensor
      // connection and let traction_manager perform the software baseline at
      // the explicit "prepare" step instead of aborting the whole hardware.
      RCLCPP_WARN(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "FT_SetZero(1) was rejected with code %d; software baseline will be captured on prepare.",
        zero_returncode);
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "FT zeroing succeeded; waiting 1.0 s for the zero to settle.");
      rclcpp::sleep_for(1s);
    }
  }
  /*
  获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
  因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
  */
  if (returncode == 0) {
    for (int j = 0; j < 6; j++) {
      _jnt_position_command[j] = jntpos.jPos[j] / 180.0 * M_PI;
    }
    RCLCPP_INFO(
      rclcpp::get_logger(
        "FairinoHardwareInterface"), "初始指令位置: %f,%f,%f,%f,%f,%f", _jnt_position_command[0], \
      _jnt_position_command[1], _jnt_position_command[2], _jnt_position_command[3],
      _jnt_position_command[4], _jnt_position_command[5]);
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");
    return hardware_interface::CallbackReturn::SUCCESS;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::return_type FairinoHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto valid_position_interface = [](const std::string & name) {
      return name.size() > 9 && name.rfind("/position") == name.size() - 9;
    };
  for (const auto & name : start_interfaces) {
    if (!valid_position_interface(name)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Unsupported command interface switch request: %s", name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  for (const auto & name : stop_interfaces) {
    if (!valid_position_interface(name)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Unsupported command interface stop request: %s", name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  if (!stop_interfaces.empty()) {
    _servoj_enabled = false;
    if (_servo_session_active) {
      const errno_t returncode = _ptr_robot->ServoMoveEnd();
      _servo_session_active = false;
      if (returncode != 0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("FairinoHardwareInterface"),
          "ServoMoveEnd failed during controller handoff, error code: %d", returncode);
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  if (!start_interfaces.empty()) {
    // The Cartesian controller synchronizes its first command to the latest
    // feedback before the first update. FR5 requires an explicit servo-session
    // handshake before the first ServoJ command; without it the SDK call blocks
    // the ros2_control loop and all Wrench/EE watchdogs expire.
    const errno_t returncode = _ptr_robot->ServoMoveStart();
    if (returncode != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "ServoMoveStart failed during controller handoff, error code: %d", returncode);
      _servoj_enabled = false;
      _servo_session_active = false;
      return hardware_interface::return_type::ERROR;
    }
    _servo_session_active = true;
    _servoj_enabled = true;
  }
  return hardware_interface::return_type::OK;
}


//停用生命周期
hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
  _servoj_enabled = false;
  if (_servo_session_active) {
    const errno_t returncode = _ptr_robot->ServoMoveEnd();
    if (returncode != 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "ServoMoveEnd failed during hardware shutdown, error code: %d", returncode);
    }
    _servo_session_active = false;
  }
  _ptr_robot->StopMotion();  //停止机器人
  _ptr_robot->CloseRPC();  //销毁实例，连接断开
  _ptr_robot.release();
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


//以配置好的频率开始循环运行
hardware_interface::return_type FairinoHardwareInterface::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
  JointPos state_data;
  error_t returncode = _ptr_robot->GetActualJointPosDegree(0, &state_data);
  //添加位置诊断
  //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "[HW I/F DEBUG] GetPos returncode: %d, Raw J1 Pos: %.4f", returncode, state_data.jPos[0]);

  if (returncode == 0) {
    for (int i = 0; i < 6; i++) {
      _jnt_position_state[i] = state_data.jPos[i] / 180.0 * M_PI;  //注意单位转换，moveit统一用弧度
      //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
    }
  } else {
    return hardware_interface::return_type::ERROR;
  }
  //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
  //     _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  // ---关键修改：直接从SDK读取关节速度 ---
  float speed_data_deg[6];   // SDK函数需要一个float数组
  errno_t speed_returncode = _ptr_robot->GetActualJointSpeedsDegree(0, speed_data_deg);
  //添加速度诊断
  // RCLCPP_INFO(
  //     rclcpp::get_logger("FairinoHardwareInterface"),
  //     "[HW I/F DEBUG] GetSpeed returncode: %d, Raw J1 Speed: %.4f",
  //     speed_returncode, speed_data_deg[0]
  // );

  if (speed_returncode == 0) {
    for (int i = 0; i < 6; i++) {
      // 将单位从 deg/s 转换为 rad/s
      _jnt_velocity_state[i] = speed_data_deg[i] / 180.0 * M_PI;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read joint speeds.");
    return hardware_interface::return_type::ERROR;
  }

  // ---读取力/力矩传感器数据
  ForceTorque ft_data;
  errno_t ft_returncode = _ptr_robot->FT_GetForceTorqueRCS(1, &ft_data);
  //添加力诊断
  // RCLCPP_INFO(
  //     rclcpp::get_logger("FairinoHardwareInterface"),
  //     "[HW I/F DEBUG] GetFT returncode: %d, Raw Fx: %.4f",
  //     ft_returncode, ft_data.fx
  // );

  if (ft_returncode == 0 && std::isfinite(ft_data.fx) && std::isfinite(ft_data.fy) &&
    std::isfinite(ft_data.fz) && std::isfinite(ft_data.tx) &&
    std::isfinite(ft_data.ty) && std::isfinite(ft_data.tz))
  {
    _ft_sensor_state[0] = ft_data.fx;
    _ft_sensor_state[1] = ft_data.fy;
    _ft_sensor_state[2] = ft_data.fz;
    _ft_sensor_state[3] = ft_data.tx;
    _ft_sensor_state[4] = ft_data.ty;
    _ft_sensor_state[5] = ft_data.tz;
    _consecutive_ft_failures = 0;
  } else {
    ++_consecutive_ft_failures;
    RCLCPP_WARN(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Force/torque read failed or returned non-finite data (%u consecutive failures); "
      "retaining the last valid sample.", _consecutive_ft_failures);
    if (_consecutive_ft_failures >= 3) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Force/torque sensor read failed three consecutive times; returning ERROR.");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;

}

//在read之后，controllers会将计算后的新命令存入_jnt_position_command
hardware_interface::return_type FairinoHardwareInterface::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  if (!_servoj_enabled) {
    // No active position controller owns the command interfaces. Keep reading
    // real feedback for the teach pendant and never hold the old joint pose.
    return hardware_interface::return_type::OK;
  }
  if (_control_mode == 0) {//位置控制模式
    if (!all_finite(_jnt_position_command, 6)) {
      return hardware_interface::return_type::ERROR;
    }
    JointPos cmd;
    ExaxisPos extcmd{0, 0, 0, 0};
    for (auto j = 0; j < 6; j++) {
      cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180;   //注意单位转换
    }
    //检查下发的关节移动指令
    //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",\
    //             cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);

    const auto servo_started = std::chrono::steady_clock::now();
    int returncode = _ptr_robot->ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);
    const double servo_duration_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - servo_started).count();
    if (servo_duration_ms > 50.0) {
      double maximum_tracking_error_deg = 0.0;
      for (int j = 0; j < 6; ++j) {
        maximum_tracking_error_deg = std::max(
          maximum_tracking_error_deg,
          std::abs(cmd.jPos[j] - _jnt_position_state[j] / M_PI * 180.0));
      }
      RCLCPP_WARN(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Slow ServoJ call: %.1f ms, max command/feedback delta: %.4f deg.",
        servo_duration_ms, maximum_tracking_error_deg);
    }
    if (returncode != 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ指令下发错误,错误码:%d",
        returncode);
      return hardware_interface::return_type::ERROR;
    }
  } else if (_control_mode == 1) {//扭矩控制模式
    if (!all_finite(_jnt_torque_command, 6)) {
      return hardware_interface::return_type::ERROR;
    }
    //_ptr_robot->write(_jnt_torque_command);//注意单位转换
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  fairino_hardware::FairinoHardwareInterface,
  hardware_interface::SystemInterface)
