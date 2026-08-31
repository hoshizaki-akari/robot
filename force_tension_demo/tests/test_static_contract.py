from __future__ import annotations

import re
import unittest
from html.parser import HTMLParser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class IdParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.ids: set[str] = set()

    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if key == "id":
                self.ids.add(value)


class StaticContractTest(unittest.TestCase):
    def test_every_javascript_id_exists_in_html(self):
        html = (ROOT / "static" / "index.html").read_text(encoding="utf-8")
        script = (ROOT / "static" / "app.js").read_text(encoding="utf-8")
        parser = IdParser()
        parser.feed(html)
        referenced = set(re.findall(r'\$\("([A-Za-z][A-Za-z0-9_-]*)"\)', script))
        self.assertEqual(referenced - parser.ids, set())

    def test_page_contains_no_robot_motion_control(self):
        html = (ROOT / "static" / "index.html").read_text(encoding="utf-8")
        self.assertNotIn("MoveL", html)
        self.assertNotIn("ServoJ", html)
        self.assertNotIn("开始运动", html)
        self.assertIn("不发送机械臂运动", html)


if __name__ == "__main__":
    unittest.main()
