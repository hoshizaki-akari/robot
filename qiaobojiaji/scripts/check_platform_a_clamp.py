#!/usr/bin/env python3
from __future__ import annotations

import json
from urllib.error import URLError
from urllib.request import urlopen


URL = "http://127.0.0.1:8765/api/platform-a/clamp/plan"


def main() -> None:
    print("平台 A 第一阶段夹挤检查（只看结果，不会控制机械臂）")
    try:
        with urlopen(URL, timeout=3) as response:
            result = json.load(response)
    except (OSError, URLError, ValueError) as error:
        raise SystemExit(f"检查失败：共同数据服务没有正常运行：{error}")

    print(result.get("message", "没有返回说明"))
    if not result.get("heel_detected"):
        print("请把足跟摆回夹挤工位，让足跟和针出现在相机画面中。")
        return
    print(f"足跟把握度：{float(result.get('heel_confidence', 0)) * 100:.1f}%")
    print(f"足跟画面中心：{result.get('heel_center_px')}")
    print(f"左右夹取点：{result.get('clamp_contact_a_px')} / {result.get('clamp_contact_b_px')}")
    if result.get("puncture_detected"):
        print(f"针孔画面位置：{result.get('puncture_px')}")
        print(f"针孔确定办法：{result.get('puncture_method')}")
        if result.get("puncture_stable"):
            print(f"针孔稳定性：通过（典型抖动 {result.get('puncture_jitter_mm')} mm）")
        else:
            print("针孔稳定性：尚未通过，请保持不动几秒后再检查。")
    else:
        print("针孔尚未找到，请让针和足跟交界处无遮挡、无强烈反光。")
    if result.get("heel_center_camera_mm") is None:
        print("深度位置尚未稳定，请保持足模型不动几秒后再检查。")
    else:
        print(f"足跟相机坐标(mm)：{result.get('heel_center_camera_mm')}")
        print(f"针孔相机坐标(mm)：{result.get('puncture_camera_mm')}")
        print(f"足跟夹取宽度(mm)：{result.get('heel_width_mm')}")
        if result.get("calibration_validated"):
            print(f"夹取中心基座坐标(mm)：{result.get('clamp_contact_center_base_mm')}")
            print(f"夹取方向（基座坐标）：{result.get('clamp_axis_base')}")
            print(f"针孔基座坐标(mm)：{result.get('puncture_base_mm')}")
            print("相机坐标已转换为机械臂基座坐标；尚未发送运动命令。")
    print("安全状态：没有发送任何机械臂或夹爪运动命令。")


if __name__ == "__main__":
    main()
