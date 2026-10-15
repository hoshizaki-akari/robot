"""Opt-in read-only AUBO i5 device status; no traction controller is started."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration("start_robot")
    sensor = LaunchConfiguration("start_sensor")
    serial_path = LaunchConfiguration("serial_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_robot", default_value="false"),
            DeclareLaunchArgument("start_sensor", default_value="false"),
            DeclareLaunchArgument("serial_path", default_value="/dev/ttyUSB0"),
            LogInfo(
                msg="AUBO i5 read-only status: no traction manager or motion controller."
            ),
            Node(
                package="aubo_v4_driver",
                executable="aubo_v4_state_node",
                name="aubo_v4_state_node",
                output="screen",
                condition=IfCondition(robot),
            ),
            Node(
                package="aubo_kwr75_sensor",
                executable="kwr75_sensor_node",
                name="kwr75_sensor_node",
                output="screen",
                parameters=[{"serial_path": serial_path}],
                condition=IfCondition(sensor),
            ),
        ]
    )
