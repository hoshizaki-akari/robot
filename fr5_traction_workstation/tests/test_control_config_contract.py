import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROS = ROOT / "ros2_overlay" / "src" / "fr_traction"


class ControlConfigurationContractTest(unittest.TestCase):
    def test_software_travel_limits_are_removed(self):
        manager = (ROS / "src" / "traction_manager_node.cpp").read_text(encoding="utf-8")
        safety = (ROS / "src" / "traction_safety.cpp").read_text(encoding="utf-8")
        driver = (ROS / "scripts" / "fr5_direct_driver_node.py").read_text(encoding="utf-8")
        parameters = (ROS / "config" / "traction_params.yaml").read_text(encoding="utf-8")
        self.assertNotIn("axial_travel_limit_m", manager + safety + parameters)
        self.assertNotIn("return_max_distance_mm", driver)
        self.assertIn("pretension_max_travel_m", manager)
        self.assertIn("tension_search_max_mm", driver)

    def test_persistent_zero_and_motion_rate_are_configured(self):
        launch = (ROS / "launch" / "traction_system.launch.py").read_text(encoding="utf-8")
        driver = (ROS / "scripts" / "fr5_direct_driver_node.py").read_text(encoding="utf-8")
        self.assertIn('"motion_rate_hz": 100.0', launch)
        self.assertIn('"update_rate_hz": 100.0', launch)
        self.assertIn("1.0 / self._motion_rate_hz, self._tick", driver)
        self.assertNotIn("1.0 / self._rate_hz, self._tick", driver)
        self.assertNotIn("now - self._last_motion_at < self._motion_period_s", driver)
        self.assertIn("cmdT=self._motion_period_s", driver)
        self.assertIn('"fixed_zero_pose_mm_deg"', launch)
        self.assertIn("500.7035522460938", launch)
        self.assertIn('"zero_pose_file"', driver)
        self.assertIn("_load_zero_pose", driver)
        self.assertIn("_save_zero_pose", driver)
        self.assertIn("os.replace", driver)

    def test_driver_bounds_command_rpc_and_return_waits_for_stop_handoff(self):
        driver = (ROS / "scripts" / "fr5_direct_driver_node.py").read_text(encoding="utf-8")
        bridge = (ROOT / "backend" / "ros_bridge.py").read_text(encoding="utf-8")
        launch = (ROS / "launch" / "traction_system.launch.py").read_text(encoding="utf-8")
        self.assertIn("command_rpc_timeout_s", driver)
        self.assertIn("startup_rpc_timeout_s", driver)
        self.assertIn("allow_existing_force_reference", driver)
        self.assertIn("_TimeoutTransport", driver)
        self.assertIn('"startup_rpc_timeout_s": 10.0', launch)
        self.assertIn('"allow_existing_force_reference": True', launch)
        self.assertIn('"return_zero_pose": 8.0', bridge)

    def test_emergency_button_controls_real_fr5_enable_state(self):
        driver = (ROS / "scripts" / "fr5_direct_driver_node.py").read_text(
            encoding="utf-8"
        )
        bridge = (ROOT / "backend" / "ros_bridge.py").read_text(encoding="utf-8")
        app = (ROOT / "app.py").read_text(encoding="utf-8")
        self.assertIn('"/traction/hardware_emergency_stop"', driver)
        self.assertIn('"/traction/hardware_emergency_recover"', driver)
        self.assertIn('(\"StopMotion\", ())', driver)
        self.assertIn('(\"RobotEnable\", (0,))', driver)
        self.assertIn('(\"ResetAllError\", ())', driver)
        self.assertIn('(\"RobotEnable\", (1,))', driver)
        self.assertIn('"hardware_emergency_stop"', bridge)
        self.assertIn('"hardware_emergency_recover"', bridge)
        self.assertIn('_call("hardware_emergency_stop")', app)
        self.assertIn('_call("hardware_emergency_recover")', app)

    def test_assisted_drag_uses_tool_frame_commands_and_independent_signs(self):
        driver = (ROS / "scripts" / "fr5_direct_driver_node.py").read_text(encoding="utf-8")
        controller = (ROS / "src" / "traction_controller_node.cpp").read_text(encoding="utf-8")
        core = (ROS / "src" / "traction_controller_core.cpp").read_text(encoding="utf-8")
        parameters = (ROS / "config" / "traction_params.yaml").read_text(encoding="utf-8")
        self.assertIn("TractionCommand.DRAG", driver)
        self.assertIn("servo_mode = 2 if", driver)
        self.assertIn("command_sign = self._base_servo_sign if servo_mode == 1 else 1.0", driver)
        self.assertIn("rotate_base_to_tool", controller)
        self.assertIn("drag_sign_x/y/z", controller)
        self.assertIn("drag_sign_x_ * wrench.x", core)
        self.assertIn("drag_sign_y_ * wrench.y", core)
        self.assertIn("drag_sign_z_ * wrench.z", core)
        self.assertIn("drag_sign_x: -1.0", parameters)
        self.assertIn("drag_sign_y: -1.0", parameters)
        self.assertIn("drag_sign_z: 1.0", parameters)
        self.assertIn("drag_force_filter_cutoff_hz: 12.0", parameters)
        self.assertIn("drag_start_force_n: 0.5", parameters)
        self.assertIn("drag_release_force_n: 0.2", parameters)
        self.assertIn("drag_gain_mps_per_n: 0.015", parameters)
        self.assertIn("drag_max_acceleration_mps2: 0.60", parameters)

    def test_assisted_drag_runs_inside_fr5_controller_with_stationary_deadzone(self):
        driver = (ROS / "scripts" / "fr5_direct_driver_node.py").read_text(
            encoding="utf-8"
        )
        manager = (ROS / "src" / "traction_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        launch = (ROS / "launch" / "traction_system.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("EndForceDragControl", driver)
        self.assertIn('"/traction/native_drag_start"', driver)
        self.assertIn('"/traction/native_drag_stop"', driver)
        self.assertIn("if self._native_drag_active:", driver)
        self.assertIn("native_drag_start_client_", manager)
        self.assertIn("native_drag_stop_client_", manager)
        self.assertIn("OperationMode::ASSISTED_DRAG", manager)
        self.assertIn(
            '"native_drag_threshold": [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]',
            launch,
        )
        self.assertNotIn("native_drag_bias_margin_n", launch + driver)
        self.assertNotIn("threshold > 10.0", driver)
        self.assertIn("SetForceSensorDragAutoFlag(0)", driver)
        self.assertIn(
            '"native_drag_damping": [120.0, 120.0, 120.0, 5.0, 5.0, 1.0]',
            launch,
        )

    def test_traction_flips_only_tool_z_before_base_frame_output(self):
        controller = (ROS / "src" / "traction_controller_node.cpp").read_text(encoding="utf-8")
        parameters = (ROS / "config" / "traction_params.yaml").read_text(encoding="utf-8")
        self.assertIn("rotate_tool_to_base", controller)
        self.assertIn("traction_sign_x_ * tool_velocity.x", controller)
        self.assertIn("traction_sign_y_ * tool_velocity.y", controller)
        self.assertIn("traction_sign_z_ * tool_velocity.z", controller)
        self.assertIn("traction_sign_x: 1.0", parameters)
        self.assertIn("traction_sign_y: 1.0", parameters)
        self.assertIn("traction_sign_z: -1.0", parameters)

    def test_all_three_modes_have_ros_interfaces(self):
        service = (ROS / "srv" / "SetOperationMode.srv").read_text(encoding="utf-8")
        status = (ROS / "msg" / "TractionStatus.msg").read_text(encoding="utf-8")
        for name in ("CONSTANT_FORCE", "POSITION_TRACTION", "ASSISTED_DRAG"):
            self.assertIn(name, service)
            self.assertIn(name, status)

    def test_position_mode_has_independent_predictive_control_and_diagnostics(self):
        command = (ROS / "msg" / "TractionCommand.msg").read_text(encoding="utf-8")
        diagnostics = (ROS / "msg" / "PositionControlDiagnostics.msg").read_text(
            encoding="utf-8"
        )
        controller = (ROS / "src" / "traction_controller_core.cpp").read_text(
            encoding="utf-8"
        )
        manager = (ROS / "src" / "traction_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        parameters = (ROS / "config" / "traction_params.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("POSITIONING=5", command)
        self.assertIn("predicted_force_n", diagnostics)
        self.assertIn("estimated_stiffness_n_per_m", diagnostics)
        self.assertIn("position_controller_.update", controller)
        self.assertIn("position_config.maximum_speed_mps = max_speed_mps_", (
            ROS / "src" / "traction_controller_node.cpp"
        ).read_text(encoding="utf-8"))
        self.assertIn("TractionCommand::POSITIONING", manager)
        self.assertIn("PositionControlDiagnostics::SETTLING", manager)
        self.assertIn("position_tolerance_n: 1.0", parameters)
        self.assertIn("position_reached_tolerance_n: 1.0", parameters)
        self.assertIn("position_reached_confirm_s: 1.0", parameters)
        self.assertIn("traction_max_speed_mps: 0.020", parameters)
        self.assertIn("position_far_gain_mps_per_n: 0.0060", parameters)
        self.assertIn("position_near_gain_mps_per_n: 0.0045", parameters)
        self.assertIn("position_prediction_horizon_s: 0.15", parameters)
        self.assertIn("position_settling_speed_mps: 0.0005", parameters)

    def test_constant_force_uses_independent_predictive_control(self):
        controller = (ROS / "src" / "traction_controller_core.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("continuous_force_controller_.update", controller)
        self.assertIn("constant_force_config(", controller)
        self.assertIn("config.maximum_speed_mps = maximum_speed_mps", controller)
        self.assertIn(
            "smooth_velocity(desired_velocity, dt_s, false)", controller
        )

    def test_web_runtime_defaults_to_current_overlay(self):
        launcher = (ROOT / "run_workstation.sh").read_text(encoding="utf-8")
        preflight = (ROOT / "scripts" / "preflight_check.sh").read_text(encoding="utf-8")
        self.assertIn('$PROJECT_DIR/ros2_overlay', launcher)
        self.assertIn('$PROJECT_DIR/ros2_overlay', preflight)
        self.assertIn('install/setup.bash', launcher)
        self.assertNotIn('/home/zhj/', launcher)
        self.assertNotIn('runtimes/directional_correction_v1', launcher)

    def test_tablet_deployment_pins_tested_fairino_sdk(self):
        setup = (ROOT / "scripts" / "tablet_setup_env.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'SDK_REVISION="62add0e6b7c7d2e157beaaa4485042c9d54e1f84"',
            setup,
        )
        self.assertIn('fetch --depth 1 origin "$SDK_REVISION"', setup)
        self.assertIn('checkout -q --detach FETCH_HEAD', setup)


if __name__ == "__main__":
    unittest.main()
