from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Sequence

import numpy as np

from .handeye_calibration import pose_to_matrix, rotation_angle_degrees
from .tool_center_calibration import flange_pose_for_center, matrix_to_rpy_degrees, point_from_pose


@dataclass(frozen=True)
class ClampMotionConfig:
    clamp_displacement_mm: float = 12.0
    # 视觉给出的是足跟可见外表面；夹爪手指中部需要继续进入到实际夹持深度。
    surface_to_grip_center_mm: float = 0.0
    image_left_correction_mm: float = 0.0
    pre_approach_mm: float = 60.0
    near_approach_mm: float = 20.0
    # 针和针线是后期人工添加物，不参与夹持规划。
    maximum_axis_error_deg: float = 12.0
    maximum_orientation_change_deg: float = 35.0
    # D435 在当前硅胶足模型上的实测平面拟合误差约 3.0～3.5 mm。
    maximum_plane_rmse_mm: float = 5.0
    minimum_plane_inliers: int = 70
    maximum_contact_plane_error_mm: float = 1.0
    maximum_lab_tcp_error_mm: float = 4.0
    force_limit_n: float = 80.0
    torque_limit_nm: float = 8.0
    gripper_force_level: int = 80
    gripper_speed_percent: int = 10


def _vector(values: Sequence[float], name: str, length: int = 3) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name}不是完整的{length}个数字")
    return result


def _workspace_check(pose: Sequence[float], name: str) -> None:
    x, y, z = map(float, pose[:3])
    if not (100.0 <= x <= 600.0):
        raise ValueError(f"{name}的X={x:.1f} mm超出实验工作区")
    if not (-600.0 <= y <= 200.0):
        raise ValueError(f"{name}的Y={y:.1f} mm超出实验工作区")
    if not (150.0 <= z <= 700.0):
        raise ValueError(f"{name}的Z={z:.1f} mm过低或过高")


def build_motion_plan(
    vision_plan: dict[str, Any],
    observation_flange_pose_mm_deg: Sequence[float],
    tcp_calibration: dict[str, Any],
    handeye_calibration: dict[str, Any],
    config: ClampMotionConfig | None = None,
) -> dict[str, Any]:
    """把稳定视觉结果变成三个分段目标；这里只计算，不控制机械臂。"""
    cfg = config or ClampMotionConfig()
    if not vision_plan.get("valid"):
        raise ValueError("足跟夹持目标还没有连续稳定")
    if not vision_plan.get("calibration_validated"):
        raise ValueError("相机到机械臂的标定没有通过")

    raw_surface_center = _vector(vision_plan["clamp_contact_center_base_mm"], "足跟表面夹取中心")
    clamp_axis = _vector(vision_plan["clamp_axis_base"], "夹取方向")
    clamp_axis /= np.linalg.norm(clamp_axis)
    # 视觉中的contact_a->contact_b就是画面左->右。正的修正值表示夹爪向画面左移。
    image_right_axis = clamp_axis.copy()
    surface_center = raw_surface_center - image_right_axis * cfg.image_left_correction_mm
    approach_axis = _vector(vision_plan["heel_plane_normal_base"], "足跟平面垂直方向")
    approach_axis /= np.linalg.norm(approach_axis)

    plane_rmse = float(vision_plan.get("heel_plane_rmse_mm", 999.0))
    plane_inliers = int(vision_plan.get("heel_plane_inlier_count", 0))
    if plane_rmse > cfg.maximum_plane_rmse_mm or plane_inliers < cfg.minimum_plane_inliers:
        raise ValueError(
            f"足跟平面深度质量不足：有效点{plane_inliers}个，误差{plane_rmse:.2f} mm"
        )
    plane_point = _vector(vision_plan["heel_plane_point_base_mm"], "足跟平面中心")
    contact_a = _vector(vision_plan["clamp_contact_a_base_mm"], "左夹持点")
    contact_b = _vector(vision_plan["clamp_contact_b_base_mm"], "右夹持点")
    contact_plane_error = max(
        abs(float(np.dot(contact_a - plane_point, approach_axis))),
        abs(float(np.dot(contact_b - plane_point, approach_axis))),
    )
    if contact_plane_error > cfg.maximum_contact_plane_error_mm:
        raise ValueError(
            f"左右夹持点没有落在同一平面，最大偏差{contact_plane_error:.2f} mm"
        )

    observation_pose = _vector(observation_flange_pose_mm_deg, "观察位姿", length=6)
    rotation = pose_to_matrix(observation_pose.tolist())[:3, :3]
    gripper_closing_axis = rotation[:, 0]
    if float(np.dot(approach_axis, rotation[:, 2])) < 0.0:
        approach_axis = -approach_axis
    if float(np.dot(clamp_axis, gripper_closing_axis)) < 0.0:
        clamp_axis = -clamp_axis
    clamp_axis = clamp_axis - float(np.dot(clamp_axis, approach_axis)) * approach_axis
    clamp_axis /= np.linalg.norm(clamp_axis)
    gripper_height_axis = np.cross(approach_axis, clamp_axis)
    gripper_height_axis /= np.linalg.norm(gripper_height_axis)
    clamp_axis = np.cross(gripper_height_axis, approach_axis)
    clamp_axis /= np.linalg.norm(clamp_axis)
    # 只沿末端伸入方向采用人工确认的深度；左右位置仍完全来自视觉。
    center = surface_center + approach_axis * cfg.surface_to_grip_center_mm
    target_rotation = np.column_stack((clamp_axis, gripper_height_axis, approach_axis))
    target_rpy = matrix_to_rpy_degrees(target_rotation)
    orientation_change_deg = rotation_angle_degrees(rotation.T @ target_rotation)
    if orientation_change_deg > cfg.maximum_orientation_change_deg:
        raise ValueError(
            f"为了垂直足跟需要转动{orientation_change_deg:.1f}度，超过安全上限"
        )
    alignment = float(np.clip(abs(np.dot(gripper_closing_axis, clamp_axis)), 0.0, 1.0))
    axis_error_deg = float(np.degrees(np.arccos(alignment)))
    # “align”阶段本来就会先转到 target_rpy；当前夹爪朝向和目标朝向不同
    # 不能作为“无法靠近”的理由。该差值只记录在计划中供界面显示。

    width_mm = float(vision_plan["heel_width_mm"])
    if not 20.0 <= width_mm <= 90.0:
        raise ValueError(f"视觉得到的足跟宽度{width_mm:.1f} mm不可信")
    final_opening_mm = width_mm - cfg.clamp_displacement_mm
    if final_opening_mm < 0.0:
        raise ValueError("夹挤位移大于足跟宽度")

    tcp_offset = _vector(
        tcp_calibration["flange_to_gripper_center_mm"], "夹爪中心偏移"
    )
    tcp_validation_error = float(
        (tcp_calibration.get("validation") or {}).get("max_error_mm", 999.0)
    )
    if tcp_validation_error > cfg.maximum_lab_tcp_error_mm:
        raise ValueError(
            f"夹爪中心误差{tcp_validation_error:.2f} mm超过实验模型上限"
        )
    handeye_error = float(
        (handeye_calibration.get("validation") or {}).get(
            "max_translation_error_mm", 999.0
        )
    )
    if not handeye_calibration.get("validated"):
        raise ValueError("相机标定没有通过独立检查")
    combined_error = handeye_error + tcp_validation_error
    if cfg.near_approach_mm <= combined_error + 5.0:
        raise ValueError("近距离停靠余量不足以覆盖标定误差")

    rpy = target_rpy
    current_gripper_center = point_from_pose(observation_pose.tolist(), tcp_offset)
    current_plane_gap = -float(np.dot(current_gripper_center - surface_center, approach_axis))
    if current_plane_gap < 15.0:
        raise ValueError(
            f"当前手指中点离足跟平面只有{current_plane_gap:.1f} mm，不能重新规划"
        )
    alignment_clearance = min(350.0, current_plane_gap)
    stages: dict[str, Any] = {}
    for name, distance in (
        ("align", alignment_clearance),
        ("pre", cfg.pre_approach_mm),
        ("near", cfg.near_approach_mm),
    ):
        gripper_center = surface_center - approach_axis * distance
        flange_pose = flange_pose_for_center(gripper_center, rpy, tcp_offset)
        _workspace_check(flange_pose, name)
        stages[name] = {
            "distance_from_target_mm": distance,
            "gripper_center_base_mm": [round(float(v), 3) for v in gripper_center],
            "flange_pose_mm_deg": [round(float(v), 6) for v in flange_pose],
        }
    final_center = center
    final_pose = flange_pose_for_center(final_center, rpy, tcp_offset)
    _workspace_check(final_pose, "contact_center")
    stages["contact_center"] = {
        "distance_from_target_mm": 0.0,
        "distance_from_surface_mm": round(-cfg.surface_to_grip_center_mm, 3),
        "gripper_center_base_mm": [round(float(v), 3) for v in final_center],
        "flange_pose_mm_deg": [round(float(v), 6) for v in final_pose],
    }

    return {
        "valid": True,
        "motion_allowed_for_lab_model": True,
        "human_use_allowed": False,
        "config": asdict(cfg),
        "heel_width_mm": round(width_mm, 3),
        "predicted_contact_opening_mm": round(width_mm, 3),
        "requested_clamp_displacement_mm": round(cfg.clamp_displacement_mm, 3),
        "predicted_final_opening_mm": round(final_opening_mm, 3),
        "raw_visual_surface_center_base_mm": [round(float(v), 3) for v in raw_surface_center],
        "visual_surface_center_base_mm": [round(float(v), 3) for v in surface_center],
        "image_right_axis_base": [round(float(v), 6) for v in image_right_axis],
        "image_left_correction_mm": round(cfg.image_left_correction_mm, 3),
        "clamp_center_base_mm": [round(float(v), 3) for v in center],
        "surface_to_grip_center_mm": round(cfg.surface_to_grip_center_mm, 3),
        "clamp_axis_base": [round(float(v), 6) for v in clamp_axis],
        "approach_axis_base": [round(float(v), 6) for v in approach_axis],
        "axis_error_deg": round(axis_error_deg, 3),
        "orientation_change_deg": round(float(orientation_change_deg), 3),
        "heel_plane_rmse_mm": round(plane_rmse, 3),
        "heel_plane_inlier_count": plane_inliers,
        "contact_plane_error_mm": round(contact_plane_error, 3),
        "target_rpy_deg": [round(float(v), 6) for v in target_rpy],
        "finger_mid_height_aligned": True,
        "combined_position_error_mm": round(combined_error, 3),
        "current_plane_gap_mm": round(current_plane_gap, 3),
        "alignment_clearance_mm": round(alignment_clearance, 3),
        "stages": stages,
        "message": "仅允许足模型实验；视觉在表面外完成对中，最后沿锁定方向进入人工确认的夹持深度",
    }
