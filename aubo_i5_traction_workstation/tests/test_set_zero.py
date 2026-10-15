import unittest
from types import SimpleNamespace
from unittest.mock import patch

from fastapi import HTTPException

import app


class SetZeroTest(unittest.TestCase):
    def test_admin_can_persist_zero_only_while_stopped(self):
        request = SimpleNamespace(headers={"x-role": "%E7%AE%A1%E7%90%86%E5%91%98"})
        snapshot = {"traction": {"state": 5}}
        with patch.object(app.bridge, "snapshot", return_value=snapshot):
            with patch.object(app, "_call", return_value={"success": True}) as invoke:
                self.assertEqual({"success": True}, app.set_zero(request))
        invoke.assert_called_once_with("set_zero_pose")

    def test_non_admin_or_active_motion_is_rejected(self):
        doctor = SimpleNamespace(headers={"x-role": "%E5%8C%BB%E7%94%9F"})
        with self.assertRaises(HTTPException) as denied:
            app.set_zero(doctor)
        self.assertEqual(403, denied.exception.status_code)

        admin = SimpleNamespace(headers={"x-role": "%E7%AE%A1%E7%90%86%E5%91%98"})
        with patch.object(app.bridge, "snapshot", return_value={"traction": {"state": 6}}):
            with self.assertRaises(HTTPException) as active:
                app.set_zero(admin)
        self.assertEqual(409, active.exception.status_code)


if __name__ == "__main__":
    unittest.main()
