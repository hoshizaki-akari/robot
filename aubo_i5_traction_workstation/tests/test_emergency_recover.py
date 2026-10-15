import unittest
from unittest.mock import call, patch

from fastapi import HTTPException

import app


class EmergencyRecoverTest(unittest.TestCase):
    def test_recovery_is_disabled_without_verified_v4_procedure(self):
        with patch.object(app, "_call") as invoke, self.assertRaises(HTTPException) as caught:
            app.emergency_recover()
        self.assertEqual(409, caught.exception.status_code)
        invoke.assert_not_called()

    def test_hardware_stop_failure_still_attempts_software_latch(self):
        failure = HTTPException(status_code=409, detail="AUBO stop unavailable")
        with patch.object(app, "_call", side_effect=[failure, {"success": True}]) as invoke, self.assertRaises(HTTPException) as caught:
            app.emergency_stop()
        self.assertIs(caught.exception, failure)
        self.assertEqual(
            [call("hardware_emergency_stop"), call("emergency_stop")],
            invoke.call_args_list,
        )


if __name__ == "__main__":
    unittest.main()
