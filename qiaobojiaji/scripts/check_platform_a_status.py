#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UI_PATH = ROOT / "platform_a" / "calcaneus_robot" / "ui.py"


def main() -> int:
    source = UI_PATH.read_text(encoding="utf-8")
    for original_group in (
        "病例与任务",
        "规划与安全参数",
        "复位辅助控制",
        "二维术野与机器人运动示意",
        "六维力/力矩实时监测",
        "位置与流程状态",
        "事件日志",
    ):
        assert original_group in source
    for added_interface in (
        "_device_panel",
        "mode_combo",
        "真实设备检查（只看，不控制）",
        "data_status_var",
    ):
        assert added_interface not in source
    assert 'os.environ.get("PLATFORM_A_DEVICE_MODE", "real")' in source
    assert 'getattr(self.controller.device,"camera_frame_png",None)' in source
    assert "等待 D435 真实画面" in source
    assert "YOLO" not in source
    print("PASS：平台 A 保持最初界面结构，原二维术野区域接收 D435 画面")
    print("PASS：夹取位置和方向尚未绘制，留待后续 YOLO")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
