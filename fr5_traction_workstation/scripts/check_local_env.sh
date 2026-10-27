#!/usr/bin/env bash
# Offline inspection only: no ROS nodes, SDK connections or network changes.
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for setup in /opt/ros/humble/setup.bash "$PROJECT_DIR/ros2_overlay/install/setup.bash"; do
  [[ -r "$setup" ]] || { echo "[FAIL] 缺少 $setup"; exit 1; }
  source "$setup"
done
VENV_DIR="${FR5_VENV_DIR:-$PROJECT_DIR/../.venv}"
[[ -x "$VENV_DIR/bin/python" ]] || { echo "[FAIL] 缺少 $VENV_DIR/bin/python"; exit 1; }
export PYTHONDONTWRITEBYTECODE=1
"$VENV_DIR/bin/python" -B - "$PROJECT_DIR" <<'PY'
import importlib
import importlib.metadata as metadata
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

root = Path(sys.argv[1]).resolve()
overlay = root / "ros2_overlay"
failures = []

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def run(*args):
    try:
        return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT, timeout=30)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(error.output.strip() or str(error)) from error

def check(label, action):
    try:
        action()
        print(f"[OK] {label}")
    except Exception as error:
        failures.append(label)
        print(f"[FAIL] {label}: {error}")

def system():
    release = dict(line.split("=", 1) for line in Path("/etc/os-release").read_text().splitlines() if "=" in line)
    require(release.get("ID", "").strip('"') == "ubuntu" and release.get("VERSION_ID", "").strip('"') == "22.04", "需要 Ubuntu 22.04")
    require(platform.machine() == "x86_64" and sys.version_info[:2] == (3, 10), "需要 x86_64 / Python 3.10")
    for name in ("bash", "git", "curl", "fuser", "timeout", "colcon", "cmake", "g++", "ldd", "xacro", "ss"):
        require(shutil.which(name), f"缺少命令 {name}")
    require("include-system-site-packages = true" in (Path(sys.prefix) / "pyvenv.cfg").read_text(), "venv 应包含系统 ROS Python 包")

def dependencies():
    for line in (root / "requirements.txt").read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            name, expected = line.split("==", 1)
            require(metadata.version(name) == expected, f"{name} 需要 {expected}，实际 {metadata.version(name)}")
            importlib.import_module(name.replace("-", "_"))
    print("  " + run(sys.executable, "-m", "pip", "check").strip())

prefixes = {}
def ros_interfaces():
    from ament_index_python.packages import get_package_prefix
    from rclpy.type_support import check_for_type_support
    for module in ("launch", "launch_ros.actions", "launch_ros.parameter_descriptions"):
        importlib.import_module(module)
    for package in ("fr_traction", "fairino_msgs", "fairino_description", "fairino5_v6_moveit2_config"):
        prefixes[package] = Path(get_package_prefix(package))
        require(prefixes[package].resolve() == (overlay / "install" / package).resolve(), f"{package} 路径错误: {prefixes[package]}")
    for package in ("robot_state_publisher", "launch_ros", "xacro"):
        get_package_prefix(package)
    interfaces = {"fr_traction.msg": "TractionCommand TractionStatus TractionHistory TractionRecordSummary PositionControlDiagnostics", "fr_traction.srv": "SetTargetForce SetOperationMode", "fairino_msgs.msg": "PoseTwist", "geometry_msgs.msg": "Twist WrenchStamped", "sensor_msgs.msg": "JointState", "std_msgs.msg": "Bool Empty", "std_srvs.srv": "Trigger", "controller_manager_msgs.srv": "SwitchController"}
    for module, names in interfaces.items():
        imported = importlib.import_module(module)
        for name in names.split():
            check_for_type_support(getattr(imported, name))
    print(f"  fr_traction: {prefixes['fr_traction']}")

def installed_files():
    prefix = prefixes.get("fr_traction", overlay / "install/fr_traction")
    for directory in ("launch", "config", "msg", "srv"):
        for source in (overlay / "src/fr_traction" / directory).glob("*"):
            if source.is_file():
                require(source.read_bytes() == (prefix / "share/fr_traction" / directory / source.name).read_bytes(), f"安装文件不同步: {source}")
    source = overlay / "src/fr_traction/scripts/fr5_direct_driver_node.py"
    require(source.read_bytes() == (prefix / "lib/fr_traction" / source.name).read_bytes(), "已安装 driver 与源码不同")
    for relative in ("lib/fr_traction/traction_manager_node", "lib/fr_traction/traction_controller_node", "lib/libfr_traction_core.so"):
        target = prefix / relative
        require(target.is_file() and os.access(target, os.X_OK), f"不可执行/缺少 {target}")
        require("not found" not in run("ldd", str(target)), f"动态库缺失: {target}")

def sdk():
    sdk_root = root / "vendor/fairino-python-sdk"
    require(run("git", "-C", str(sdk_root), "rev-parse", "HEAD").strip() == "62add0e6b7c7d2e157beaaa4485042c9d54e1f84", "SDK 提交不匹配")
    spec = importlib.util.spec_from_file_location("fr5_offline_driver", overlay / "src/fr_traction/scripts/fr5_direct_driver_node.py")
    driver = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(driver)
    robot = driver._load_robot_sdk(str(sdk_root / "linux"))
    for method in ("FT_SetRCS", "FT_Activate", "ServoCart", "ServoMoveStart", "ServoMoveEnd", "EndForceDragControl", "Mode", "RobotEnable"):
        require(callable(getattr(robot.RPC, method, None)), f"SDK 缺少 {method}")

def description():
    config = overlay / "install/fairino5_v6_moveit2_config/share/fairino5_v6_moveit2_config/config"
    tree = ET.fromstring(run("xacro", str(config / "fairino5_v6_robot.urdf.xacro"), f"initial_positions_file:={config / 'initial_positions.yaml'}"))
    require(tree.tag == "robot" and len(tree.findall("joint")) >= 6, "URDF 展开不完整")

def resources():
    assets = ["090105.html", "090105.js", "robot_viewer.js", "vendor/three.module.js", "vendor/OrbitControls.js", "vendor/STLLoader.js"]
    assets += [f"model/fairino5_v6/{name}_link.STL" for name in ("base", "shoulder", "upperarm", "forearm", "wrist1", "wrist2", "wrist3")]
    for asset in assets:
        require((root / "static" / asset).stat().st_size > 0, f"缺少静态资源 {asset}")
    data = Path(os.environ.get("FR5_TRACTION_DATA_DIR", root / "debug/traction_sessions")).expanduser()
    while not data.exists():
        data = data.parent
    require(os.access(data, os.W_OK | os.X_OK), f"数据目录不可写: {data}")

print("FR5 本机离线环境检查（不连接机器人、不启动 ROS 节点）")
for label, action in (("系统与工具", system), ("Python 依赖和 pip check", dependencies), ("ROS 路径、接口与类型支持", ros_interfaces), ("安装文件与动态库", installed_files), ("固定 SDK 与安全导入", sdk), ("机器人描述展开", description), ("网页资源与数据目录权限", resources)):
    check(label, action)
for name in ("FR5_ROS_WS", "FR5_VENV_DIR", "FR5_SDK_PYTHON_PATH", "FR5_ROBOT_IP", "FR5_TRACTION_DATA_DIR", "WORKSTATION_PORT", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY"):
    if name in os.environ:
        print(f"[WARN] 环境覆盖 {name}={os.environ[name]}；两个启动终端须保持一致")
for name in ("zero_pose.json", "settings.json"):
    path = Path.home() / ".local/state/fr5-traction" / name
    if path.exists():
        print(f"[WARN] 已有持久配置 {path}；现场核对后使用")
        try:
            value = json.loads(path.read_text())
            if name == "zero_pose.json":
                require(len(value["pose_mm_deg"]) == 6 and all(math.isfinite(float(v)) for v in value["pose_mm_deg"]), "零位必须含六个有限数")
        except Exception as error:
            print(f"[WARN] 配置无效: {error}")
try:
    if run("ss", "-H", "-ltn", "sport", "=", os.environ.get("WORKSTATION_PORT", "8081")).strip():
        print("[WARN] 网页端口已占用；启动脚本会终止其占用进程，请先核对")
except Exception as error:
    print(f"[WARN] 无法读取端口占用: {error}")
print("LOCAL_ENV_READY：仅本机软件环境就绪，现场仍需验证网络、传感器和真实反馈。" if not failures else f"LOCAL_ENV_NOT_READY：{len(failures)} 项必需检查失败。")
sys.exit(bool(failures))
PY
