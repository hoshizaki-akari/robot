from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .handeye_calibration import pose_to_matrix


CALIBRATION_FILE = Path(__file__).resolve().parent / "config" / "handeye_calibration.json"


def _transform_point(base_t_camera: np.ndarray, point_mm: list[float]) -> list[float]:
    point_m = np.ones(4, dtype=np.float64)
    point_m[:3] = np.asarray(point_mm, dtype=np.float64) / 1000.0
    result = base_t_camera @ point_m
    return [round(float(value * 1000.0), 2) for value in result[:3]]


def _transform_vector(base_t_camera: np.ndarray, vector: list[float]) -> list[float]:
    result = base_t_camera[:3, :3] @ np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if norm < 1e-9:
        raise ValueError("足跟平面法向量长度为零")
    return [round(float(value / norm), 7) for value in result]


def build_clamp_plan(vision: dict[str, Any], fr5: dict[str, Any]) -> dict[str, Any]:
    """把相机坐标下的识别结果转换到机械臂基座坐标。

    这里只做坐标计算，不生成或发送运动命令。夹爪相对法兰的安装偏移
    仍需现场确认，因此 motion_allowed 保持 False。
    """
    result = dict(vision)
    result["motion_allowed"] = False
    result["coordinate_system"] = "camera"
    if not vision.get("valid"):
        return result
    if not CALIBRATION_FILE.is_file():
        result["message"] = "识别有效，但还没有相机标定文件"
        return result
    try:
        calibration = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
        if not calibration.get("validated"):
            raise ValueError("相机标定尚未通过验收")
        if not fr5.get("valid") or int(fr5.get("age_ms", 999999999)) > 1500:
            raise ValueError("机械臂位置不是实时数据")
        flange_pose = fr5.get("flange_pose_mm_deg") or []
        if len(flange_pose) != 6:
            raise ValueError("没有实时法兰位置")
        base_t_flange = pose_to_matrix(
            [float(value) for value in flange_pose],
            calibration.get("euler_convention", "rpy"),
        )
        flange_t_camera = np.asarray(calibration["flange_T_camera"], dtype=np.float64)
        base_t_camera = base_t_flange @ flange_t_camera
        result["base_T_camera"] = base_t_camera.tolist()
        for name in (
            "heel_center_camera_mm",
            "clamp_contact_a_camera_mm",
            "clamp_contact_b_camera_mm",
            "heel_plane_point_camera_mm",
        ):
            point = vision.get(name)
            if point is not None:
                result[name.replace("_camera_mm", "_base_mm")] = _transform_point(
                    base_t_camera, point
                )
        plane_normal = vision.get("heel_plane_normal_camera")
        if plane_normal is None:
            raise ValueError("没有足跟局部平面方向")
        result["heel_plane_normal_base"] = _transform_vector(
            base_t_camera, plane_normal
        )
        point_a = result.get("clamp_contact_a_base_mm")
        point_b = result.get("clamp_contact_b_base_mm")
        if point_a and point_b:
            camera_a = result.get("clamp_contact_a_camera_mm")
            camera_b = result.get("clamp_contact_b_camera_mm")
            if camera_a and camera_b:
                # D435Monitor 对左右接触点分别做了多帧稳定；夹持中心必须由
                # 这对稳定点派生，不能混用单帧 heel_center_camera_mm。
                result["clamp_contact_center_camera_mm"] = [
                    round((float(a) + float(b)) / 2.0, 2)
                    for a, b in zip(camera_a, camera_b)
                ]
            result["clamp_contact_center_base_mm"] = [
                round((float(a) + float(b)) / 2.0, 2) for a, b in zip(point_a, point_b)
            ]
            axis = np.asarray(point_b, dtype=np.float64) - np.asarray(point_a, dtype=np.float64)
            normal = np.asarray(result["heel_plane_normal_base"], dtype=np.float64)
            axis = axis - float(np.dot(axis, normal)) * normal
            norm = float(np.linalg.norm(axis))
            if norm > 1e-6:
                result["clamp_axis_base"] = [round(float(value / norm), 5) for value in axis]
        result["calibration_validated"] = True
        result["motion_allowed"] = False
        result["message"] = "已找到足跟局部平面、左右夹持点和针孔；等待生成垂直靠近路线"
        return result
    except (OSError, KeyError, TypeError, ValueError) as error:
        # 坐标变换不完整时，不能把原始视觉的 valid 状态直接透传给运动入口。
        result["valid"] = False
        result["motion_allowed"] = False
        result["calibration_validated"] = False
        result["message"] = f"识别完成，但坐标转换失败：{error}"
        return result
