import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 新增：声明一个启动参数来设定恒力大小 ---
    declare_force_arg = DeclareLaunchArgument(
        'target_force_magnitude',
        default_value='30.0', # 默认值为10N
        description='Magnitude of the constant force to apply in Newtons.'
    )
    # declare_use_external_rviz = DeclareLaunchArgument(
    #     'use_external_rviz', 
    #     default_value='true',
    #     description='Launch a separate rviz2 process'
    # )

    # --- 配置文件路径 ---
    constantforce_params_file = os.path.join(get_package_share_directory("fairino_admittance"), 'config', 'ConstantForce_param.yaml')
    controller_yaml_path = os.path.join(get_package_share_directory("fairino5_v6_moveit2_config"), "config", 'admittance_controller.yaml')

    # --- 加载机器人模型 (与原launch文件相同) ---
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(file_path="config/fairino5_v6_robot.urdf.xacro")
        # !! 重要 !! 加载包含ros2_control硬件接口定义的xacro
        .robot_description_semantic(file_path="config/fairino5_v6_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # --- 启动核心节点 (与原launch文件相同) ---
    robot_state_publisher_node = Node(        
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],)
    
    ros2_control_node = Node(        
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controller_yaml_path
        ],
        output="screen",)
    
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

    # --- 核心修改：启动新的 AdmittanceForceNode ---
    admittance_force_node = Node(
        package="fairino_admittance",
        executable="ConstantForce_node", # <-- 使用新的可执行文件
        name="ConstantForce",      # <-- 使用新的节点名称
        parameters=[
                    constantforce_params_file,
                    {'target_force_magnitude':LaunchConfiguration('target_force_magnitude')}
                    ],
        output="screen",
    )
##################################################################################
    # 启动RViz
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
        #condition=IfCondition(LaunchConfiguration('use_external_rviz'))
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

    
################################################################################
    return LaunchDescription([
        declare_force_arg, # <-- 添加声明
        robot_state_publisher_node,
        ros2_control_node,
        spawn_controllers,
        admittance_force_node, # <-- 启动新节点
        rviz_node,
        move_group_node,
        force_visualizer_node,
    ])