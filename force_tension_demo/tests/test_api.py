from __future__ import annotations

import os
import tempfile
import unittest

os.environ["FORCE_DEMO_DISABLE_SOURCES"] = "1"

from fastapi.testclient import TestClient

from app import app
from force_demo.classifier import SensorSample


class ApiTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        os.environ["FORCE_DEMO_DEBUG_DIR"] = self.temp.name
        self.client_context = TestClient(app)
        self.client = self.client_context.__enter__()

    def tearDown(self) -> None:
        self.client_context.__exit__(None, None, None)
        self.temp.cleanup()

    def test_page_and_health_are_available(self):
        response = self.client.get("/")
        self.assertEqual(response.status_code, 200)
        self.assertIn("KWR75D 绳带松紧检测", response.text)
        health = self.client.get("/api/health").json()
        self.assertTrue(health["ok"])
        self.assertFalse(health["sensor_connected"])

    def test_baseline_requires_real_fresh_data(self):
        response = self.client.post("/api/baseline")
        self.assertEqual(response.status_code, 409)
        self.assertIn("真实传感器数据", response.json()["detail"])

    def test_api_accepts_injected_real_sample_and_exports_csv(self):
        engine = app.state.engine
        engine.ingest(SensorSample.create(
            (1.0, 2.0, 3.0), source="ros2",
            source_detail="/force_torque_sensor_broadcaster/wrench",
            priority=30, motion_available=True,
        ))
        state = self.client.get("/api/state").json()
        self.assertTrue(state["connected"])
        self.assertEqual(state["frame_id"], "base_link")
        self.assertEqual(self.client.post("/api/baseline").status_code, 200)
        exported = self.client.get("/api/export/latest")
        self.assertEqual(exported.status_code, 200)
        self.assertTrue(exported.text.startswith("wall_time,source"))

    def test_websocket_streams_state(self):
        with self.client.websocket_connect("/ws") as websocket:
            state = websocket.receive_json()
        self.assertIn("phase", state)
        self.assertIn("connected", state)

    def test_direction_reverse_endpoint(self):
        before = app.state.engine.snapshot()["increase_direction_sign"]
        response = self.client.post("/api/direction/reverse")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["increase_direction_sign"], -before)


if __name__ == "__main__":
    unittest.main()
