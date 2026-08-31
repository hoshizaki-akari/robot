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
        self.assertIn("/ws", script)
        self.assertNotIn("Math.random()", script)
        self.assertNotIn("actualForce +=", script)

    def test_no_old_platform_b_dependency(self):
        page = (ROOT / "static" / "090105.html").read_text(encoding="utf-8")
        self.assertNotIn("platform_b/", page)


if __name__ == "__main__":
    unittest.main()
