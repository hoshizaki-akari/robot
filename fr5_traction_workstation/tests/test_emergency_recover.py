import unittest
from unittest.mock import call, patch

from fastapi import HTTPException

import app


class EmergencyRecoverTest(unittest.TestCase):
    def test_retries_transient_manager_health_lag(self):
        transient = HTTPException(
            status_code=409,
            detail="Reset rejected: FR5 must be stationary and reporting fresh data.",
        )
        with patch.object(app, "_call", side_effect=[{"success": True}, transient, {"success": True}]) as invoke:
            with patch.object(app.time, "sleep") as sleep:
                self.assertEqual({"success": True}, app.emergency_recover())
        self.assertEqual(
            [call("hardware_emergency_recover"), call("reset_fault"), call("reset_fault")],
            invoke.call_args_list,
        )
        sleep.assert_called_once_with(0.15)

    def test_hardware_failure_never_clears_manager_latch(self):
        failure = HTTPException(status_code=409, detail="FR5 emergency stop input is still active.")
        with patch.object(app, "_call", side_effect=failure) as invoke:
            with self.assertRaises(HTTPException):
                app.emergency_recover()
        invoke.assert_called_once_with("hardware_emergency_recover")


if __name__ == "__main__":
    unittest.main()
