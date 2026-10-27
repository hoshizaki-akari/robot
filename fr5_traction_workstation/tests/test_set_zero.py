"""Regression coverage for removing zero-pose motion from the workstation API."""

import unittest
from unittest.mock import patch

import app


class RemovedZeroPoseTest(unittest.TestCase):
    def test_zero_pose_endpoints_are_removed(self):
        routes = {route.path for route in app.app.routes}
        self.assertNotIn("/api/traction/set-zero", routes)
        self.assertNotIn("/api/traction/return-zero", routes)
        self.assertNotIn("/api/traction/calibrate-direction", routes)

    def test_unload_calls_manager_service(self):
        with patch.object(app, "_call", return_value={"success": True}) as invoke:
            self.assertEqual({"success": True}, app.unload())
        invoke.assert_called_once_with("unload")


if __name__ == "__main__":
    unittest.main()
