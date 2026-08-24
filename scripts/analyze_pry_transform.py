import json
import numpy as np
from platform_a.handeye_calibration import pose_to_matrix
from platform_a.tool_center_calibration import point_from_pose

ROOT = __import__('pathlib').Path(__file__).resolve().parents[1]
h = json.loads((ROOT/'platform_a/config/handeye_calibration.json').read_text())
tcp = json.loads((ROOT/'platform_a/config/gripper_tcp_calibration.json').read_text())
home = json.loads((ROOT/'platform_a/config/pry_home_position.json').read_text())['flange_pose_mm_deg']
manual = [-496.2340088, -638.053833, 285.3470764, -88.62525, 2.6341, 127.63424]
camera_center = np.array([-136.625, 42.61, 395.504]) / 1000.0
Tbc = pose_to_matrix(home) @ np.array(h['flange_T_camera'])
expected_center_base = point_from_pose(manual, tcp['flange_to_gripper_center_mm']) / 1000.0
observed_center_base = (Tbc @ np.r_[camera_center, 1])[:3]
expected_camera = np.linalg.inv(Tbc) @ np.r_[expected_center_base, 1]
print(json.dumps({'observed_center_base_mm': (observed_center_base*1000).tolist(), 'expected_center_base_mm': (expected_center_base*1000).tolist(), 'expected_camera_from_manual_mm': (expected_camera[:3]*1000).tolist(), 'camera_error_mm': ((expected_camera[:3]-camera_center)*1000).tolist()}, indent=2))
