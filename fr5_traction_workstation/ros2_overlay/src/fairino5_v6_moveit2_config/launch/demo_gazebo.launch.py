import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # --- 1. 定义功能包名称 ---
    # 这是包含您的URDF/XACRO文件和此启动文件的包
    moveit_config_pkg_name = "fairino5_v6_moveit2_config"
    # 这是包含您的AdmittanceNode的包
    admittance_pkg_name = "fairino_admittance"

    # --- 2. 从XACRO加载机器人描述 ---
    # 这是现代且正确的方式。它会自动处理您所有的xacro include，
    # 包括我们之前添加的gazebo和ros2_control插件。
    robot_description = MoveItConfigsBuilder(
        "fairino5_v6_robot", package_name=moveit_config_pkg_name
    ).to_dict()

    # --- 3. 启动Gazebo ---
    # 我们使用IncludeLaunchDescription来启动Gazebo服务器和客户端
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
    )

    # --- 4. 在Gazebo中生成机器人模型 ---
    # 这个节点订阅robot_description话题，并在Gazebo中生成实体
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'fairino5_v6_robot'],
        output='screen'
    )

    # --- 5. 启动Robot State Publisher ---
    # 这个节点根据joint_states发布TF变换树
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]  # 将机器人描述字典作为参数传递
    )

    # --- 6. 启动控制器Spawners ---
    # 这些节点现在可以连接到由Gazebo插件运行的/controller_manager服务
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["cartesian_velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["force_torque_sensor_broadcaster", "-c", "/controller_manager"],
            output="screen",
        ),
    ]

    # --- 7. 启动导纳节点 ---
    admittance_params_path = os.path.join(
        get_package_share_directory(admittance_pkg_name),
        "config",
        "AdmittanceParam.yaml"
    )
    admittance_node = Node(
        package=admittance_pkg_name,
        executable="Admittance_node",
        name="Admittance_node",
        parameters=[admittance_params_path],
        output="screen",
    )

    # --- 8. 组装并返回LaunchDescription ---
    # 我们将所有节点一起启动。
    return LaunchDescription([
        gazebo,
        spawn_entity_node,
        robot_state_publisher_node,
        admittance_node,
    ] + controller_spawners)