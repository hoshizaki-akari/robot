import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---------------------------
    # Launch Arguments
    # ---------------------------
    declare_force_arg = DeclareLaunchArgument(
        "target_force_magnitude",
        default_value="10.0",
        description="Magnitude of the constant force to apply in Newtons."
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2"
    )

    declare_spawn_delay = DeclareLaunchArgument(
        "spawn_delay",
        default_value="3.0",
        description="Delay (s) before spawning ros2_control controllers"
    )

    # ---------------------------
    # Paths
    # ---------------------------
    fuzzy_params_file = os.path.join(
        get_package_share_directory("fairino_admittance"),
        "config",
        "FuzzyConstantForce.yaml",
    )

    controller_yaml_path = os.path.join(
        get_package_share_directory("fairino5_v6_moveit2_config"),
        "config",
        "admittance_controller.yaml",
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("fairino5_v6_moveit2_config"),
        "launch",
        "moveit.rviz",
    )

    # ---------------------------
    # MoveIt config (same as constant_force.launch.py)
    # ---------------------------
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(file_path="config/fairino5_v6_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/fairino5_v6_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # ---------------------------
    # Core nodes
    # ---------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, controller_yaml_path],
        output="screen",
    )

    # ---------------------------
    # Spawn controllers (delayed)
    # ---------------------------
    spawn_controllers = TimerAction(
        period=LaunchConfiguration("spawn_delay"),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["cartesian_velocity_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["force_torque_sensor_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ],
    )

    # ---------------------------
    # Fuzzy Constant Force Node (the only difference)
    # ---------------------------
    fuzzy_force_node = Node(
        package="fairino_admittance",
        executable="FuzzyConstantForce_node",
        name="FuzzyConstantForce",
        output="screen",
        parameters=[
            fuzzy_params_file,
            {"target_force_magnitude": LaunchConfiguration("target_force_magnitude")},
        ],
    )

    # ---------------------------
    # RViz (optional)
    # ---------------------------
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        {"tf_buffer_duration": 2},
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=rviz_parameters,
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ---------------------------
    # Move Group
    # ---------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ---------------------------
    # Force visualizer (same)
    # ---------------------------
    force_visualizer_node = Node(
        package="fr_force",
        executable="force_visualizer",
        name="force_visualizer",
        output="screen",
        parameters=[{"force_scale": 0.05}],
    )

    # ---------------------------
    # LaunchDescription
    # ---------------------------
    return LaunchDescription([
        declare_force_arg,
        declare_use_rviz,
        declare_spawn_delay,

        robot_state_publisher_node,
        ros2_control_node,
        spawn_controllers,

        fuzzy_force_node,

        rviz_node,
        move_group_node,
        force_visualizer_node,
    ])
