import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 您的包名，请务必替换！
    # my_package_name1 = "fairino5_v6_moveit2_config"
    
    # 找到包含配置文件的目录
    # pkg_share1 = get_package_share_directory(my_package_name1)
    admittance_params_file = os.path.join(get_package_share_directory("fairino_admittance"), 'config', 'AdmittanceParam.yaml')
    controller_yaml_path = os.path.join(get_package_share_directory("fairino5_v6_moveit2_config"), "config", 'admittance_controller.yaml')
    # --- 1. 加载机器人模型和ros2_control配置 ---
    # 假设您的MoveIt配置包名为 fairino5_v6_moveit2_config
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(file_path="config/fairino5_v6_robot.urdf.xacro")
        # !! 重要 !! 加载包含ros2_control硬件接口定义的xacro
        .robot_description_semantic(file_path="config/fairino5_v6_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # --- 2. 启动核心节点 ---
    # a. robot_state_publisher: 发布TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    
    # b. ros2_control_node: 加载controller_manager和硬件接口
    #    这通常在您的机器人专属的ros2_control启动文件中定义
    #    这里我们假设它已经存在并能被调用
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controller_yaml_path
        ],
        output="screen",
    )

    # c. 加载并启动控制器
    spawn_controllers = TimerAction(
        period = 3.0,
        actions=[Node(
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
        )
        ]
    )

    # b. 启动导纳控制节点
    admittance_node = Node(
        package= "fairino_admittance",
        executable="Admittance_node", 
        name="Admittance_node",
        parameters=[admittance_params_file],
        output="screen",
    )
#################################################################################3
    # rviz2
    # 统一声明和使用 use_sim_time
    # declare_use_sim_time_argument = DeclareLaunchArgument(
    #     "use_sim_time",
    #     default_value="false",
    #     description="Use simulation (Gazebo) clock if true",
    # )
    # use_sim_time = LaunchConfiguration("use_sim_time")

    # # 启动RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("fairino5_v6_moveit2_config"),
        "launch",
        "moveit.rviz",
    )
    
    # # --- 关键修复：为RViz节点添加TF缓冲参数 ---
    # # 创建一个包含RViz所有参数的字典
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    force_visualizer_node = Node(
        package="fr_force",
        executable="force_visualizer",
        name="force_visualizer",
        output="screen",
        parameters=[{'force_scale': 0.05}]
    )    


    return LaunchDescription(
        [
            robot_state_publisher_node,
            ros2_control_node,
            admittance_node,
            spawn_controllers,
            rviz_node,
            move_group_node,
            force_visualizer_node,
        ]
    )