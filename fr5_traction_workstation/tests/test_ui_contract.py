import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class UiContractTest(unittest.TestCase):
    def test_clean_page_exists_and_uses_real_endpoints(self):
        page = (ROOT / "static" / "090105.html").read_text(encoding="utf-8")
        script = (ROOT / "static" / "090105.js").read_text(encoding="utf-8")
        self.assertIn("robotViewer", page)
        self.assertIn("forceCanvas", page)
        self.assertIn("/api/traction", script)
        self.assertIn("回零", page)
        self.assertIn("初始校准", page)
        self.assertIn("方向确定", page)
        self.assertIn("开始牵引", page)
        self.assertIn("结束牵引", page)
        self.assertIn("急停", page)
        self.assertIn("方向稳定", page)
        self.assertIn("正在跟随方向", script)
        self.assertEqual(len(re.findall(r'class="action-btn', page)), 6)
        self.assertIn("<th>原因</th>", page)
        self.assertNotIn("尚未感知到有效张力", page)
        self.assertIn("/api/traction/return-zero", script)
        self.assertNotIn("设置零点", page)
        self.assertNotIn("setZeroBtn", script)
        self.assertNotIn("resetBtn", page)
        self.assertIn("Always send the value currently shown", script)
        self.assertIn("finishRequested", script)
        self.assertIn("/ws", script)
        self.assertNotIn("Math.random()", script)
        self.assertNotIn("actualForce +=", script)
        self.assertIn("previousForce", script)

    def test_no_old_platform_b_dependency(self):
        page = (ROOT / "static" / "090105.html").read_text(encoding="utf-8")
        self.assertNotIn("platform_b/", page)


if __name__ == "__main__":
    unittest.main()
