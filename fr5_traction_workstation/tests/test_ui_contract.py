import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class UiContractTest(unittest.TestCase):
    def test_page_exposes_current_modes_and_compact_calibration(self):
        page = (ROOT / "static" / "090105.html").read_text(encoding="utf-8")
        mode_ids = re.findall(r'class="mode-btn[^\"]*" data-mode="([0-9]+)"', page)
        self.assertEqual(["2", "1", "3"], mode_ids)
        for label in ("省力拖拽", "目标牵引", "卸力", "重新校准"):
            self.assertIn(label, page)
        for removed in ("恒力牵引", "位置牵引", "方向确定", "回零", "设置当前位置为零位"):
            self.assertNotIn(removed, page)
        self.assertIn('class="nav-btn" id="prepareBtn"', page)
        self.assertEqual(page.count('id="prepareBtn"'), 1)
        self.assertEqual(page.count('class="action-slot"'), 3)
        action_ids = re.findall(r'class="action-btn[^\"]*" id="([^\"]+)"', page)
        self.assertEqual(["startBtn", "stopBtn", "unloadBtn", "emergencyBtn"], action_ids)
        self.assertIn('id="settingForceLimit" type="number" min="1" step="1"', page)
        self.assertIn('id="targetForceVal" type="range"', page)
        self.assertIn("robotViewer", page)
        self.assertIn("forceCanvas", page)
        self.assertIn("/assets/090105.js?v=20261027-target-unload", page)

    def test_real_services_history_and_shutdown_remain(self):
        script = (ROOT / "static" / "090105.js").read_text(encoding="utf-8")
        page = (ROOT / "static" / "090105.html").read_text(encoding="utf-8")
        for endpoint in ("unload", "prepare", "start", "stop", "emergency-recover", "mode"):
            self.assertIn(f"/api/traction/{endpoint}", script)
        for removed in ("/api/traction/return-zero", "/api/traction/set-zero", "calibrateBtn"):
            self.assertNotIn(removed, script)
        self.assertIn("/api/traction/export/session/", script)
        self.assertIn("/api/system/shutdown", script)
        self.assertIn("FORCE_HISTORY_WINDOW_MS = 60000", script)
        self.assertIn("context.setLineDash([9, 7])", script)
        self.assertIn("导出全部摘要", page)
        self.assertNotIn("Math.random()", script)
        self.assertNotIn("platform_b/", page)

    def test_dynamic_base_to_tool_rotation_uses_live_tcp_orientation(self):
        tool_to_base = (
            (-0.693470, -0.024221, 0.720078),
            (-0.719703, 0.069842, -0.690760),
            (-0.033561, -0.997264, -0.065866),
        )

        def multiply(matrix, vector):
            return tuple(sum(row[i] * vector[i] for i in range(3)) for row in matrix)

        base_to_tool = tuple(zip(*tool_to_base))
        for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            recovered = multiply(base_to_tool, multiply(tool_to_base, axis))
            for actual, expected in zip(recovered, axis):
                self.assertAlmostEqual(actual, expected, places=5)
        viewer = (ROOT / "static" / "robot_viewer.js").read_text(encoding="utf-8")
        self.assertIn(".applyQuaternion(baseToToolQuaternion)", viewer)
        self.assertIn(".applyQuaternion(physicalToolToModelQuaternion)", viewer)


if __name__ == "__main__":
    unittest.main()
