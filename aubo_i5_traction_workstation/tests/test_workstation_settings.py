import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from fastapi import HTTPException

import app


class WorkstationSettingsTest(unittest.TestCase):
    def test_force_limit_is_persisted_and_loaded(self):
        with tempfile.TemporaryDirectory() as directory:
            settings_path = Path(directory) / "settings.json"
            with patch.object(app, "SETTINGS_PATH", settings_path):
                saved = app.save_settings(app.SettingsRequest(traction_force_limit_n=72))
                self.assertEqual({"traction_force_limit_n": 72.0}, saved)
                self.assertEqual(saved, app.get_settings())

    def test_target_endpoint_enforces_configured_force_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            settings_path = Path(directory) / "settings.json"
            with patch.object(app, "SETTINGS_PATH", settings_path):
                app.save_settings(app.SettingsRequest(traction_force_limit_n=45))
                with self.assertRaises(HTTPException) as rejected:
                    app.set_target(app.TargetRequest(target_force_n=46))
                self.assertEqual(409, rejected.exception.status_code)

                with patch.object(app, "_call", return_value={"success": True}) as invoke:
                    self.assertEqual(
                        {"success": True},
                        app.set_target(app.TargetRequest(target_force_n=45)),
                    )
                invoke.assert_called_once_with("set_target_force", 45.0)


if __name__ == "__main__":
    unittest.main()
