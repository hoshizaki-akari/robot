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

    def test_fixed_zero_and_motion_rate_are_configured(self):
        launch = (ROS / "launch" / "traction_system.launch.py").read_text(encoding="utf-8")
        driver = (ROS / "scripts" / "fr5_direct_driver_node.py").read_text(encoding="utf-8")
        self.assertIn('"motion_rate_hz": 50.0', launch)
        self.assertIn('"fixed_zero_pose_mm_deg"', launch)
        self.assertIn("500.7035522460938", launch)
        self.assertIn("fixed zero pose unchanged", driver)

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
        self.assertIn("drag_sign_x: 1.0", parameters)
        self.assertIn("drag_sign_y: 1.0", parameters)
        self.assertIn("drag_sign_z: 1.0", parameters)

    def test_all_three_modes_have_ros_interfaces(self):
        service = (ROS / "srv" / "SetOperationMode.srv").read_text(encoding="utf-8")
        status = (ROS / "msg" / "TractionStatus.msg").read_text(encoding="utf-8")
        for name in ("CONSTANT_FORCE", "POSITION_TRACTION", "ASSISTED_DRAG"):
            self.assertIn(name, service)
            self.assertIn(name, status)

    def test_web_runtime_defaults_to_current_overlay(self):
        launcher = (ROOT / "run_workstation.sh").read_text(encoding="utf-8")
        preflight = (ROOT / "scripts" / "preflight_check.sh").read_text(encoding="utf-8")
        self.assertIn('$PROJECT_DIR/ros2_overlay', launcher)
        self.assertIn('$PROJECT_DIR/ros2_overlay', preflight)
        self.assertIn('install/local_setup.bash', launcher)
        self.assertIn('runtimes/directional_correction_v1', launcher)


if __name__ == "__main__":
    unittest.main()
