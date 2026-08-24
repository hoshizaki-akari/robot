from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class BoardSettings:
    marker_length_mm: float
    marker_gap_mm: float
    markers_x: int = 4
    markers_y: int = 3
    first_marker_id: int = 0


def aruco_dictionary():
    aruco = cv2.aruco
    value = aruco.DICT_5X5_50
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(value)
    return aruco.Dictionary_get(value)


def board_model(settings: BoardSettings):
    """建立与实物编号一致的 4x3 标定板。

    实物从相机看到的编号是：
    9 6 3 0 / 10 7 4 1 / 11 8 5 2。
    不能直接使用 OpenCV 默认的 0..11 排列，否则会产生约 90° 的姿态错误。
    """
    aruco = cv2.aruco
    dictionary = aruco_dictionary()
    standard = aruco.GridBoard_create(
        settings.markers_x,
        settings.markers_y,
        settings.marker_length_mm / 1000.0,
        settings.marker_gap_mm / 1000.0,
        dictionary,
        settings.first_marker_id,
    )
    actual_ids = np.asarray([9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2], dtype=np.int32)
    if hasattr(aruco, "Board_create"):
        return aruco.Board_create(list(standard.objPoints), dictionary, actual_ids)
    return standard


def detect_markers(image_bgr: np.ndarray) -> tuple[list[Any], np.ndarray | None]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco
    dictionary = aruco_dictionary()
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    # 当前 OpenCV/相机组合返回 [右上, 右下, 左下, 左上]；
    # Board 的物理点顺序是 [左上, 右上, 右下, 左下]。
    # 不统一顺序时，中心点看似正确，但姿态会偏几十度。
    corners = [np.roll(np.asarray(corner), 1, axis=1) for corner in corners]
    return corners, ids


def draw_detection(image_bgr: np.ndarray, corners: list[Any], ids: np.ndarray | None) -> np.ndarray:
    output = image_bgr.copy()
    if ids is not None and len(ids):
        cv2.aruco.drawDetectedMarkers(output, corners, ids)
        text = f"Detected IDs: {', '.join(str(int(v)) for v in ids.flatten())}"
        color = (20, 180, 20)
    else:
        text = "No complete ArUco marker detected"
        color = (0, 0, 255)
    cv2.putText(output, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    return output


def estimate_board_pose(
    image_bgr: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    settings: BoardSettings,
) -> tuple[np.ndarray, list[int], np.ndarray]:
    corners, ids = detect_markers(image_bgr)
    annotated = draw_detection(image_bgr, corners, ids)
    if ids is None:
        raise ValueError("没有看到完整的标定方块")
    wanted = set(range(settings.first_marker_id, settings.first_marker_id + 12))
    visible = [int(v) for v in ids.flatten() if int(v) in wanted]
    if len(visible) < 6:
        raise ValueError(f"只看到 {len(visible)} 个有效方块，至少要看到 6 个")
    aruco = cv2.aruco
    dictionary = aruco_dictionary()
    board = board_model(settings)
    count, rvec, tvec = aruco.estimatePoseBoard(
        corners, ids, board, camera_matrix, distortion, None, None
    )
    if int(count) < 6 or rvec is None or tvec is None:
        raise ValueError("标定板位置计算失败，请让标定板完整、清楚地出现在画面中")
    rotation, _ = cv2.Rodrigues(rvec)
    camera_t_board = np.eye(4, dtype=np.float64)
    camera_t_board[:3, :3] = rotation
    camera_t_board[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    cv2.drawFrameAxes(
        annotated,
        camera_matrix,
        distortion,
        rvec,
        tvec,
        settings.marker_length_mm / 1000.0 * 1.5,
    )
    return camera_t_board, visible, annotated


def pose_to_matrix(pose_mm_deg: list[float], convention: str = "rpy") -> np.ndarray:
    if len(pose_mm_deg) != 6:
        raise ValueError("机械臂位置必须有 6 个数字")
    x, y, z = [float(v) / 1000.0 for v in pose_mm_deg[:3]]
    rx, ry, rz = [math.radians(float(v)) for v in pose_mm_deg[3:]]
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    rx_m = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    ry_m = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rz_m = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    if convention == "rpy":
        rotation = rz_m @ ry_m @ rx_m
    elif convention == "xyz_intrinsic":
        rotation = rx_m @ ry_m @ rz_m
    else:
        raise ValueError(f"未知角度算法：{convention}")
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = [x, y, z]
    return result


def rotation_angle_degrees(rotation: np.ndarray) -> float:
    value = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(value))


def sample_distance(a: dict[str, Any], b: dict[str, Any]) -> tuple[float, float]:
    ta = np.asarray(a["base_T_flange"], dtype=np.float64)
    tb = np.asarray(b["base_T_flange"], dtype=np.float64)
    translation_mm = float(np.linalg.norm(ta[:3, 3] - tb[:3, 3]) * 1000.0)
    rotation_deg = rotation_angle_degrees(ta[:3, :3].T @ tb[:3, :3])
    return translation_mm, rotation_deg


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    u, _, vt = np.linalg.svd(sum(rotations))
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def solve(samples_raw: list[dict[str, Any]]) -> dict[str, Any]:
    if len(samples_raw) < 15:
        raise ValueError(f"现在只有 {len(samples_raw)} 个位置，至少需要 15 个")
    candidates: list[dict[str, Any]] = []
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    }
    for convention in ("rpy", "xyz_intrinsic"):
        base_t_flange = [
            pose_to_matrix(sample["flange_pose_mm_deg"], convention) for sample in samples_raw
        ]
        camera_t_board = [
            np.asarray(sample["camera_T_board"], dtype=np.float64) for sample in samples_raw
        ]
        for method_name, method_value in methods.items():
            try:
                rotation, translation = cv2.calibrateHandEye(
                    [value[:3, :3] for value in base_t_flange],
                    [value[:3, 3] for value in base_t_flange],
                    [value[:3, :3] for value in camera_t_board],
                    [value[:3, 3] for value in camera_t_board],
                    method=method_value,
                )
                flange_t_camera = np.eye(4, dtype=np.float64)
                flange_t_camera[:3, :3] = rotation
                flange_t_camera[:3, 3] = np.asarray(translation).reshape(3)
                if not np.all(np.isfinite(flange_t_camera)):
                    continue
                base_t_board = [
                    base_t_flange[index] @ flange_t_camera @ camera_t_board[index]
                    for index in range(len(samples_raw))
                ]
                translations = np.asarray([value[:3, 3] for value in base_t_board])
                center = np.mean(translations, axis=0)
                translation_errors = np.linalg.norm(translations - center, axis=1) * 1000.0
                mean_rotation = _mean_rotation([value[:3, :3] for value in base_t_board])
                rotation_errors = [
                    rotation_angle_degrees(mean_rotation.T @ value[:3, :3])
                    for value in base_t_board
                ]
                translation_rmse = float(np.sqrt(np.mean(translation_errors ** 2)))
                rotation_rmse = float(np.sqrt(np.mean(np.square(rotation_errors))))
                candidates.append(
                    {
                        "method": method_name,
                        "euler_convention": convention,
                        "flange_T_camera": flange_t_camera.tolist(),
                        "translation_rmse_mm": round(translation_rmse, 3),
                        "rotation_rmse_deg": round(rotation_rmse, 3),
                        "score": translation_rmse + rotation_rmse * 2.0,
                    }
                )
            except cv2.error:
                continue
    if not candidates:
        raise ValueError("这些观察角度无法算出结果，请增加不同方向的观察位置")
    best = min(candidates, key=lambda item: item["score"])
    best.pop("score", None)
    best.update(
        {
            "camera_mount": "eye_in_hand",
            "sample_count": len(samples_raw),
            "runtime_formula": "base_T_camera = base_T_flange @ flange_T_camera",
            "validated": False,
            "warning": "结果还要用独立位置检查，检查通过前不能用于自动运动",
        }
    )
    return best
