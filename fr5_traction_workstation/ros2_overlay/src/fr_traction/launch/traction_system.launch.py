"""Single-entry launch for the fixed-model FR5 traction prototype."""

# The direct driver is the sole FR SDK owner. MoveGroup, vision, gripper and
# the blocking ros2_control hardware path are intentionally not included.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fairino_share = get_package_share_directory("fairino5_v6_moveit2_config")
    traction_share = get_package_share_directory("fr_traction")
    urdf_file = os.path.join(fairino_share, "config", "fairino5_v6_robot.urdf.xacro")
    initial_positions_file = os.path.join(fairino_share, "config", "initial_positions.yaml")
    traction_yaml = os.path.join(traction_share, "config", "traction_params.yaml")
    rviz_config = os.path.join(fairino_share, "launch", "moveit.rviz")

    robot_ip = LaunchConfiguration("robot_ip")
    zero_sensor_on_activate = LaunchConfiguration("zero_sensor_on_activate")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_web_bridge = LaunchConfiguration("use_web_bridge")
    data_directory = LaunchConfiguration("data_directory")
    sdk_python_path = LaunchConfiguration("sdk_python_path")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                urdf_file,
                " initial_positions_file:=",
                initial_positions_file,
                " robot_ip:=",
                robot_ip,
                " zero_sensor_on_activate:=",
                zero_sensor_on_activate,
            ]),
            value_type=str,
        )
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    direct_driver = Node(
        package="fr_traction",
        executable="fr5_direct_driver_node.py",
        name="fr5_direct_driver",
        output="screen",
        parameters=[{
            "robot_ip": robot_ip,
            "sdk_python_path": sdk_python_path,
            # The FR5 SDK exposes four feedback RPCs. 25 Hz keeps the real
            # hardware feedback continuous instead of queueing 100 Hz calls.
            "update_rate_hz": 25.0,
            "motion_rate_hz": 25.0,
            # Reject a velocity command left behind by a blocked feedback RPC.
            "command_timeout_s": 0.25,
            "max_linear_speed_mps": 0.005,
            # On this FR5, the live Y- tension search showed that a positive
            # base command must be passed through unchanged for the force
            # controller to move in the measured increasing-force direction.
            "base_servo_sign": 1.0,
            "return_speed_mm_s": 2.0,
            "return_max_distance_mm": 35.0,
            "tension_search_max_mm": 30.0,
            "auto_set_zero_on_start": True,
            "use_sim_time": use_sim_time,
        }],
    )

    manager = Node(
        package="fr_traction",
        executable="traction_manager_node",
        name="traction_manager",
        output="screen",
        parameters=[
            traction_yaml, {"data_directory": data_directory, "use_sim_time": use_sim_time}
        ],
    )
    controller = Node(
        package="fr_traction",
        executable="traction_controller_node",
        name="traction_controller",
        output="screen",
        parameters=[traction_yaml, {"use_sim_time": use_sim_time}],
    )

    bridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        condition=IfCondition(use_web_bridge),
        parameters=[{"use_sim_time": use_sim_time}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
        DeclareLaunchArgument("zero_sensor_on_activate", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_web_bridge", default_value="false"),
        DeclareLaunchArgument("data_directory", default_value="debug/traction_sessions"),
        DeclareLaunchArgument(
            "sdk_python_path",
            default_value="/home/zhj/projects/fr5_learning/vendor/fairino-python-sdk/linux",
        ),
        robot_state_publisher,
        direct_driver,
        controller,
        manager,
        bridge,
        rviz,
    ])
