"""Offline contracts for the AUBO-only copy of the traction workstation."""

import unittest
from pathlib import Path
from unittest.mock import patch

from fastapi import HTTPException

import app

ROOT = Path(__file__).resolve().parents[1]
ROS = ROOT / "ros2_overlay" / "src" / "fr_traction"


class AuboWorkstationContractTest(unittest.TestCase):
    def test_original_fr5_launch_cannot_run(self):
        launch = (ROS / "launch" / "traction_system.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("raise RuntimeError(", launch)
        self.assertIn("copied FR5 traction launch is disabled", launch)
        ros_start = (ROOT / "scripts" / "start_ros_stack.sh").read_text(encoding="utf-8")
        tablet_start = (ROOT / "scripts" / "tablet_start.sh").read_text(encoding="utf-8")
        self.assertIn("aubo_i5_readonly.launch.py", ros_start)
        self.assertNotIn("traction_system.launch.py", ros_start)
        self.assertIn('run_workstation.sh', tablet_start)
        self.assertNotIn('FR5_ROBOT_IP', tablet_start)
        readonly_launch = (ROS / "launch" / "aubo_i5_readonly.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('start_robot", default_value="false"', readonly_launch)
        self.assertIn('start_sensor", default_value="false"', readonly_launch)
        self.assertNotIn("traction_controller_node", readonly_launch)

    def test_ros_package_has_no_fairino_runtime_dependency(self):
        cmake = (ROS / "CMakeLists.txt").read_text(encoding="utf-8")
        package = (ROS / "package.xml").read_text(encoding="utf-8")
        manager = (ROS / "src" / "traction_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        controller = (ROS / "src" / "traction_controller_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("fairino_msgs", cmake + package + manager + controller)
        self.assertNotIn("fr5_direct_driver_node.py", cmake)
        self.assertIn('"msg/EndEffectorState.msg"', cmake)

    def test_hardware_motion_gate_defaults_closed(self):
        manager = (ROS / "src" / "traction_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        params = (ROS / "config" / "traction_params.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn('declare_parameter("aubo_port_verified", false)', manager)
        self.assertIn("return aubo_port_verified_ && hardware_healthy_", manager)
        self.assertIn("aubo_port_verified: false", params)
        self.assertIn("AUBO_PORT_NOT_VERIFIED", manager)

    def test_fr5_axis_signs_are_not_copied(self):
        params = (ROS / "config" / "traction_params.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("traction_sign_z: 1.0", params)
        self.assertIn("drag_sign_x: 1.0", params)
        self.assertIn("drag_sign_y: 1.0", params)
        self.assertNotIn("traction_sign_z: -1.0", params)

    def test_drag_request_fails_closed(self):
        manager = (ROS / "src" / "traction_manager_node.cpp").read_text(
            encoding="utf-8"
        )
        start_handler = manager.split("void handle_start", 1)[1].split(
            "void handle_stop", 1
        )[0]
        self.assertIn("AUBO i5 assisted drag is unavailable", start_handler)
        self.assertNotIn("FR5 native drag activation requested", start_handler)

    def test_web_contract_uses_robot_identity(self):
        app = (ROOT / "app.py").read_text(encoding="utf-8")
        bridge = (ROOT / "backend" / "ros_bridge.py").read_text(encoding="utf-8")
        ui = (ROOT / "static" / "090105.js").read_text(encoding="utf-8")
        self.assertIn('"robot": robot', bridge)
        self.assertIn('snapshot["robot"]["connected"]', app)
        self.assertIn("state.robot && state.robot.joint_position_deg", ui)
        self.assertNotIn("state.fr5", ui)

    def test_web_motion_requests_fail_closed_before_ros(self):
        with patch.object(app.bridge, "call") as send:
            for service in app.MOTION_SERVICES_UNVERIFIED:
                with self.assertRaises(HTTPException) as caught:
                    app._call(service)
                self.assertEqual(409, caught.exception.status_code)
        send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
