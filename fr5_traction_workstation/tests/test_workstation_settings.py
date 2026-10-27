import asyncio
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from fastapi import HTTPException
from pydantic import ValidationError

import app
from backend.ros_bridge import RosBridgeError


class WorkstationSettingsTest(unittest.TestCase):
    def test_force_limit_above_100_is_persisted_after_manager_accepts(self):
        with tempfile.TemporaryDirectory() as directory:
            settings_path = Path(directory) / "settings.json"
            with (
                patch.object(app, "SETTINGS_PATH", settings_path),
                patch.object(app.bridge, "set_target_force_limit") as configure,
            ):
                self.assertEqual({"traction_force_limit_n": 100.0}, app.get_settings())
                saved = app.save_settings(app.SettingsRequest(traction_force_limit_n=160))
                self.assertEqual({"traction_force_limit_n": 160.0}, saved)
                self.assertEqual(saved, app.get_settings())
                configure.assert_called_once_with(160.0)

    def test_target_endpoint_enforces_configured_force_limit(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(app, "SETTINGS_PATH", Path(directory) / "settings.json"),
            patch.object(app.bridge, "set_target_force_limit"),
        ):
            app.save_settings(app.SettingsRequest(traction_force_limit_n=140))
            with self.assertRaises(HTTPException) as rejected:
                app.set_target(app.TargetRequest(target_force_n=141))
            self.assertEqual(409, rejected.exception.status_code)
            with patch.object(app, "_call", return_value={"success": True}) as invoke:
                self.assertEqual(
                    {"success": True}, app.set_target(app.TargetRequest(target_force_n=130))
                )
            invoke.assert_called_once_with("set_target_force", 130.0)

    def test_manager_rejection_does_not_save_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            path.write_text('{"traction_force_limit_n": 100}', encoding="utf-8")
            with (
                patch.object(app, "SETTINGS_PATH", path),
                patch.object(app.bridge, "set_target_force_limit", side_effect=RosBridgeError("busy")),
            ):
                with self.assertRaises(HTTPException) as rejected:
                    app.save_settings(app.SettingsRequest(traction_force_limit_n=150))
                self.assertEqual(409, rejected.exception.status_code)
                self.assertEqual({"traction_force_limit_n": 100.0}, app.get_settings())

    def test_nonfinite_settings_and_targets_are_rejected(self):
        for value in (float("inf"), float("nan"), -1):
            with self.subTest(value=value):
                with self.assertRaises(ValidationError):
                    app.SettingsRequest(traction_force_limit_n=value)
                with self.assertRaises(ValidationError):
                    app.TargetRequest(target_force_n=value)

    def test_only_current_modes_are_accepted(self):
        for mode in (1, 2, 3):
            self.assertEqual(mode, app.OperationModeRequest(mode=mode).mode)
        for mode in (0, 4):
            with self.assertRaises(ValidationError):
                app.OperationModeRequest(mode=mode)

    def test_settings_follow_manager_restarts_without_recalibration_or_redundant_writes(self):
        # The web server survives a manager restart: first it sees the saved
        # range, then stale/disconnected data, then the manager's default 100.
        snapshots = [
            {"traction": {"connected": True, "target_force_max_n": 200.0}},
            {"traction": {"connected": False, "target_force_max_n": 200.0}},
            {"traction": {"connected": True, "target_force_max_n": 100.0}},
            {"traction": {"connected": True, "target_force_max_n": 200.0}},
        ]
        with (
            patch.object(app, "_read_settings", return_value={"traction_force_limit_n": 200}),
            patch.object(app.bridge, "snapshot", side_effect=snapshots),
            patch.object(app.bridge, "set_target_force_limit") as sync,
            patch.object(app.asyncio, "sleep", side_effect=[None, None, None, asyncio.CancelledError]) as sleep,
            patch.object(app.bridge, "call") as command,
            self.assertRaises(asyncio.CancelledError),
        ):
            asyncio.run(app._sync_startup_settings())
        sync.assert_called_once_with(200)
        self.assertEqual(4, sleep.await_count)
        command.assert_not_called()

    def test_settings_sync_retries_service_failure_without_saving_or_calibrating(self):
        with (
            patch.object(app, "_read_settings", return_value={"traction_force_limit_n": 145}),
            patch.object(app.bridge, "snapshot", return_value={
                "traction": {"connected": True, "target_force_max_n": 100.0}
            }),
            patch.object(app.bridge, "set_target_force_limit", side_effect=[RosBridgeError("offline"), None]) as sync,
            patch.object(app.asyncio, "sleep", side_effect=[None, asyncio.CancelledError]) as sleep,
            patch.object(app.bridge, "call") as command,
            patch.object(app, "_write_settings") as save,
            self.assertRaises(asyncio.CancelledError),
        ):
            asyncio.run(app._sync_startup_settings())
        self.assertEqual(2, sync.call_count)
        sync.assert_called_with(145)
        self.assertEqual(2, sleep.await_count)
        command.assert_not_called()
        save.assert_not_called()


if __name__ == "__main__":
    unittest.main()
