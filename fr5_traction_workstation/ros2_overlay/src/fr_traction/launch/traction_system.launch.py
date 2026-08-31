"""Single-entry launch for the fixed-model FR5 traction prototype."""

# Only ros2_control owns the FR SDK connection. Legacy direct-SDK nodes,
# MoveGroup, vision and gripper launch files are intentionally not included.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fairino_share = get_package_share_directory("fairino5_v6_moveit2_config")
    traction_share = get_package_share_directory("fr_traction")
    urdf_file = os.path.join(fairino_share, "config", "fairino5_v6_robot.urdf.xacro")
    initial_positions_file = os.path.join(fairino_share, "config", "initial_positions.yaml")
    controller_yaml = os.path.join(fairino_share, "config", "admittance_controller.yaml")
    cartesian_controller_yaml = os.path.join(
        fairino_share, "config", "cartesian_velocity_controller.yaml"
    )
    force_torque_yaml = os.path.join(
        fairino_share, "config", "force_torque_sensor_broadcaster.yaml"
    )
    traction_yaml = os.path.join(traction_share, "config", "traction_params.yaml")
    rviz_config = os.path.join(fairino_share, "launch", "moveit.rviz")

    robot_ip = LaunchConfiguration("robot_ip")
    zero_sensor_on_activate = LaunchConfiguration("zero_sensor_on_activate")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_web_bridge = LaunchConfiguration("use_web_bridge")
    data_directory = LaunchConfiguration("data_directory")

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
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[robot_description, controller_yaml, {"use_sim_time": use_sim_time}],
    )

    spawn_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_cartesian = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_velocity_controller", "--controller-manager", "/controller_manager",
            "--inactive",
            "--param-file", cartesian_controller_yaml,
        ],
        output="screen",
    )
    spawn_force_torque = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "force_torque_sensor_broadcaster", "--controller-manager", "/controller_manager",
            "--param-file", force_torque_yaml,
        ],
        output="screen",
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

    # Controller spawners are event-chained.  Cartesian motion is deliberately
    # loaded inactive so the operator can use the official teach pendant during
    # MANUAL_SETUP without a stale ServoJ hold. The manager activates it only
    # after direction confirmation and a fresh handoff check.
    # The manager still independently
    # waits for the controller health topic before becoming READY, so a failed
    # spawner can never silently enable traction.
    start_joint = RegisterEventHandler(
        OnProcessStart(target_action=ros2_control_node, on_start=[spawn_joint_state])
    )
    start_cartesian = RegisterEventHandler(
        OnProcessExit(target_action=spawn_joint_state, on_exit=[spawn_cartesian])
    )
    start_force_torque = RegisterEventHandler(
        OnProcessExit(target_action=spawn_cartesian, on_exit=[spawn_force_torque])
    )
    start_business = RegisterEventHandler(
        OnProcessExit(target_action=spawn_force_torque, on_exit=[controller, manager])
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
        DeclareLaunchArgument("zero_sensor_on_activate", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_web_bridge", default_value="false"),
        DeclareLaunchArgument("data_directory", default_value="debug/traction_sessions"),
        robot_state_publisher,
        ros2_control_node,
        start_joint,
        start_cartesian,
        start_force_torque,
        start_business,
        bridge,
        rviz,
    ])
