"""Ordinary V4 short-move backend. Default is offline, no robot/serial access."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("fr_traction")
    config = os.path.join(share, "config", "traction_params.yaml")
    mount = os.path.join(share, "config", "aubo_wrench_mount.yaml")

    def boolean(name):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)

    arguments = [
        DeclareLaunchArgument(name, default_value="false")
        for name in (
            "connect_hardware", "enable_motion", "aubo_port_verified",
            "mounting_verified", "pose_frame_verified",
            "force_polarity_verified", "drag_axes_verified",
        )
    ]
    arguments.extend([
        DeclareLaunchArgument("serial_path", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("axis_sign", default_value="0.0"),
        DeclareLaunchArgument("wrench_calibration_file", default_value=mount),
        DeclareLaunchArgument("drag_axis_mask", default_value="[1.0, 1.0, 0.0]"),
    ])
    return LaunchDescription(arguments + [
        Node(
            package="aubo_v4_driver", executable="aubo_v4_stepwise_node",
            name="aubo_v4_state_node", output="screen",
            parameters=[{
                "connect_hardware": boolean("connect_hardware"),
                "enable_motion_interface": boolean("enable_motion"),
                "pose_frame_id": "base_link",
                # Real V4 first-step feedback differed from the IK endpoint by
                # ~0.10 mm. Use 1 mm steps and 0.2 mm arrival tolerance rather
                # than claiming 0.05 mm repeatability on this controller.
                "maximum_step_m": 0.001,
                "planning_horizon_s": 0.5,
                "arrival_tolerance_m": 0.0002,
            }],
        ),
        Node(
            package="aubo_kwr75_sensor", executable="kwr75_sensor_node",
            name="kwr75_sensor_node", output="screen",
            condition=IfCondition(LaunchConfiguration("connect_hardware")),
            parameters=[{"serial_path": LaunchConfiguration("serial_path")}],
        ),
        Node(
            package="fr_traction", executable="aubo_wrench_transform_node.py",
            name="aubo_wrench_transform", output="screen",
            parameters=[LaunchConfiguration("wrench_calibration_file"), {
                "mounting_verified": boolean("mounting_verified"),
                "pose_frame_verified": boolean("pose_frame_verified"),
                "pose_timeout_s": 0.5,
            }],
        ),
        Node(
            package="fr_traction", executable="traction_controller_node",
            name="traction_controller", output="screen",
            parameters=[config, {
                "stepwise_backend": True,
                "stepwise_force_polarity_verified": boolean("force_polarity_verified"),
                "stepwise_drag_axes_verified": boolean("drag_axes_verified"),
                "stepwise_axis_sign": ParameterValue(
                    LaunchConfiguration("axis_sign"), value_type=float),
                "stepwise_maximum_speed_mps": 0.002,
                "stepwise_drag_axis_mask": ParameterValue(LaunchConfiguration("drag_axis_mask")),
            }],
        ),
        Node(
            package="fr_traction", executable="traction_manager_node",
            name="traction_manager", output="screen",
            parameters=[config, {
                "aubo_port_verified": boolean("aubo_port_verified"),
                "require_measured_motion_idle": True,
                "return_speed_mps": 0.002,
                "direction_correction_max_speed_mps": 0.002,
                "validated_target_max_n": 5.0,
            }],
        ),
    ])
