import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    my_package_name = "fr_force"

    # 统一声明和使用 use_sim_time
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(file_path="config/fairino5_v6_robot.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 启动RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("fairino5_v6_moveit2_config"),
        "launch",
        "moveit.rviz",
    )
    
    # --- 关键修复：为RViz节点添加TF缓冲参数 ---
    # 创建一个包含RViz所有参数的字典
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        {"use_sim_time": use_sim_time},
        # 增加TF缓冲时长到2.0秒，给RViz足够的时间来查找变换
        {"tf_buffer_duration": 2},
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=rviz_parameters, # 使用我们上面定义的参数字典
    )

    ##启动 robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 启动 MoveGroup 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 启动我们自己的节点
    robot_driver_node = Node(
        package=my_package_name,
        executable="fr_robot_driver",
        name="fr_robot_driver",
        output="screen",
    )

    force_visualizer_node = Node(
        package=my_package_name,
        executable="force_visualizer",
        name="force_visualizer",
        output="screen",
        parameters=[{'force_scale': 0.05}]
    )

    return LaunchDescription(
        [
            declare_use_sim_time_argument,
            rviz_node,
            robot_state_publisher_node,
            move_group_node,
            robot_driver_node,
            force_visualizer_node,
        ]
    )