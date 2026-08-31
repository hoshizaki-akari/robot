import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 可调：恒力大小（N）
    declare_force_arg = DeclareLaunchArgument(
        "target_force_magnitude",
        default_value="20.0",
        description="Magnitude of the constant force to apply in Newtons."
    )

    # —— 路径配置 —— #
    # 自适应版参数文件（在 fairino_admittance 包里）
    adaptive_params_file = os.path.join(
        get_package_share_directory("fairino_admittance"),
        "config", "AdaptiveConstantForce.yaml"
    )
    # 控制器（与你原文件相同）
    controller_yaml_path = os.path.join(
        get_package_share_directory("fairino5_v6_moveit2_config"),
        "config", "admittance_controller.yaml"
    )

    # —— 机器人/MoveIt 配置（与你原文件一致） —— #
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(file_path="config/fairino5_v6_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/fairino5_v6_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

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

    spawn_controllers = TimerAction(
        period=3.0,
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

    # —— 关键：启动自适应变阻尼导纳控制节点 —— #
    adaptive_node = Node(
        package="fairino_admittance",
        executable="adaptive_constant_force_node",  # 新可执行文件
        name="AdaptiveConstantForce",
        output="screen",
        parameters=[
            adaptive_params_file,
            # 用启动参数覆盖YAML里的 target_force_magnitude（可选）
            {"target_force_magnitude": LaunchConfiguration("target_force_magnitude")},
        ],
    )

    # —— RViz / MoveGroup / 受力可视化（与原文件一致） —— #
    rviz_config_file = os.path.join(
        get_package_share_directory("fairino5_v6_moveit2_config"),
        "launch", "moveit.rviz",
    )
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        {"tf_buffer_duration": 2},   # 给足TF缓存
    ]
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=rviz_parameters,
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    force_visualizer_node = Node(
        package="fr_force",
        executable="force_visualizer",
        name="force_visualizer",
        output="screen",
        parameters=[{"force_scale": 0.05}],
    )

    return LaunchDescription([
        declare_force_arg,
        robot_state_publisher_node,
        ros2_control_node,
        spawn_controllers,
        adaptive_node,          # 只启动自适应节点（不要和旧的 ConstantForce 同时跑）
        rviz_node,
        move_group_node,
        force_visualizer_node,
    ])
