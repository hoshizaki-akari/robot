import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class UiContractTest(unittest.TestCase):
    def test_clean_page_exists_and_uses_real_endpoints(self):
        page = (ROOT / "static" / "090105.html").read_text(encoding="utf-8")
        script = (ROOT / "static" / "090105.js").read_text(encoding="utf-8")
        app = (ROOT / "app.py").read_text(encoding="utf-8")
        tablet_start = (ROOT / "scripts" / "tablet_start.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("robotViewer", page)
        self.assertIn("forceCanvas", page)
        self.assertIn("/api/traction", script)
        self.assertIn("回零", page)
        self.assertIn("初始校准", page)
        self.assertIn("方向确定", page)
        self.assertIn("开始牵引", page)
        self.assertIn("结束牵引", page)
        self.assertIn("急停", page)
        self.assertIn("/api/traction/emergency-recover", script)
        self.assertIn("recovering ? '恢复说明' : '急停'", script)
        self.assertIn("if (!Object.keys(users).length)", script)
        self.assertIn("省力拖拽", page)
        self.assertIn("enable_follow_drag === '1'", script)
        self.assertIn("原生 TRACKING", script)
        self.assertIn("session_limit_reached === '1'", script)
        self.assertIn("位置牵引", page)
        self.assertIn("恒力牵引", page)
        self.assertIn("/api/traction/mode", script)
        self.assertIn("方向稳定", page)
        self.assertIn("正在跟随方向", script)
        self.assertNotIn("三轴受力", page)
        self.assertNotIn("受力方向", page)
        self.assertNotIn("增力方向", page)
        self.assertEqual(len(re.findall(r'class="action-btn', page)), 6)
        self.assertIn("<th>原因</th>", page)
        self.assertNotIn("尚未感知到有效张力", page)
        self.assertIn("/api/traction/return-zero", script)
        self.assertIn("设置当前位置为零位", page)
        self.assertIn("setZeroPoseBtn", script)
        self.assertIn("/api/traction/set-zero", script)
        self.assertIn('@app.post("/api/traction/set-zero")', app)
        self.assertNotIn("resetBtn", page)
        self.assertIn("Always send the value currently shown", script)
        self.assertIn("finishRequested", script)
        self.assertIn("DRAG_COMPLETED: '拖拽已完成'", script)
        self.assertIn("POSITION_TRACTION_COMPLETED: '位置牵引已完成'", script)
        self.assertIn('id="shutdownBtn"', page)
        self.assertIn('id="shutdownModal"', page)
        self.assertIn("/api/system/shutdown", script)
        self.assertIn('@app.post("/api/system/shutdown")', app)
        self.assertIn('read-only until the AUBO ROS graph', tablet_start)
        self.assertIn('id="jointReadout"', page)
        self.assertIn("state.robot.joint_position_deg", script)
        self.assertNotIn("/static/robot_viewer.js", page)
        self.assertNotIn("/assets/vendor/three.module.js", page)
        self.assertIn("/ws", script)
        self.assertNotIn("Math.random()", script)
        self.assertNotIn("actualForce +=", script)
        self.assertIn("confirmedForce", script)
        self.assertIn("导出全部摘要", page)
        self.assertIn("data-export-session", script)
        self.assertIn("/api/traction/export/session/", script)
        self.assertIn("等待张紧", script)
        self.assertIn("方向校准成功", script)
        self.assertNotIn("最大行程", page)
        self.assertNotIn("settingTravelLimit", script)
        self.assertIn('id="targetForceVal" type="range"', page)
        self.assertIn('step="1"', page)
        self.assertNotIn('class="round-btn', page)
        self.assertEqual(page.count('class="action-slot"'), 4)
        self.assertIn("const showDirectionAction = operationMode !== 2 && tractionState === 2", script)
        self.assertIn("const showStopAction = pendingStart || [6, 7, 11, 12, 13].includes(tractionState)", script)
        self.assertIn("if (requestGeneration !== startRequestGeneration) return;", script)
        self.assertIn("13: '正在回零'", script)
        self.assertIn("const TARGET_FORCE_ABSOLUTE_MAX = 100", script)
        self.assertIn("liveTractionAdjustment", script)
        self.assertIn("FORCE_HISTORY_WINDOW_MS = 60000", script)
        self.assertIn("context.setLineDash([9, 7])", script)
        self.assertIn("traction_force_limit_n", script)
        self.assertIn('id="settingForceLimit"', page)
        self.assertIn("/api/settings", script)
        self.assertIn('@app.post("/api/settings")', app)
        action_ids = re.findall(r'class="action-btn[^"]*" id="([^"]+)"', page)
        self.assertEqual(
            action_ids,
            [
                "prepareBtn", "calibrateBtn", "startBtn",
                "stopBtn", "returnZeroBtn", "emergencyBtn",
            ],
        )
        self.assertIn('class="force-section-title">牵引力调节</div>', page)
        self.assertIn('<div class="force-metric-label">目标</div>', page)
        self.assertIn('<div class="force-metric-label">当前</div>', page)
        self.assertIn(".arm-panel .card-title { color: #fff; }", page)

    def test_no_old_platform_b_dependency(self):
        page = (ROOT / "static" / "090105.html").read_text(encoding="utf-8")
        self.assertNotIn("platform_b/", page)


if __name__ == "__main__":
    unittest.main()
