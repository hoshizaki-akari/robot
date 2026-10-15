"""AUBO i5 V4 traction chain; calibration gates remain explicit."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("fr_traction")
    control_config = os.path.join(share, "config", "traction_params.yaml")
    mount_config = os.path.join(share, "config", "aubo_wrench_mount.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_path", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("aubo_port_verified", default_value="false"),
            Node(
                package="aubo_v4_driver",
                executable="aubo_v4_state_node",
                name="aubo_v4_state_node",
                output="screen",
                parameters=[
                    {"enable_motion_interface": True, "pose_frame_id": "base_link"}
                ],
            ),
            Node(
                package="aubo_kwr75_sensor",
                executable="kwr75_sensor_node",
                name="kwr75_sensor_node",
                output="screen",
                parameters=[{"serial_path": LaunchConfiguration("serial_path")}],
            ),
            Node(
                package="fr_traction",
                executable="aubo_wrench_transform_node.py",
                name="aubo_wrench_transform",
                output="screen",
                parameters=[mount_config],
            ),
            Node(
                package="fr_traction",
                executable="traction_controller_node",
                name="traction_controller",
                output="screen",
                parameters=[control_config],
            ),
            Node(
                package="fr_traction",
                executable="traction_manager_node",
                name="traction_manager",
                output="screen",
                parameters=[
                    control_config,
                    {
                        "aubo_port_verified": ParameterValue(
                            LaunchConfiguration("aubo_port_verified"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
