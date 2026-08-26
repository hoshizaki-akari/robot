#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import copy
import json
import os
import subprocess
import sys
import threading
import time
import uuid
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from time import monotonic
from typing import Any
from urllib.request import Request, urlopen

import uvicorn
from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

try:
    from .robot_control import (
        RobotControlError,
        dispatch as robot_dispatch,
        load_params,
        save_params,
        set_gripper_opening,
    )
except ImportError:
    from robot_control import (
        RobotControlError,
        dispatch as robot_dispatch,
        load_params,
        save_params,
        set_gripper_opening,
    )



UPSTREAM_URL = os.environ.get(
    "FR5_STATE_URL", "http://127.0.0.1:8765/api/state"
)
ROOT = Path(__file__).resolve().parent
workflow_lock = threading.Lock()
workflow_state: dict[str, Any] = {
    "active": False,
    "branch": None,
    "stage": "idle",
    "message": "等待用户确认",
    "returncode": None,
}
pry_service_process: subprocess.Popen[str] | None = None
PRY_SERVICE_URL = os.environ.get("FR5_PRY_VISION_URL", "http://127.0.0.1:8766")
CLAMP_CAPTURE_MAX_AGE_S = 30.0
clamp_capture_lock = Lock()
clamp_capture: dict[str, Any] | None = None
clamp_capture_at = 0.0


class UpstreamCache:
    def __init__(self) -> None:
        self._lock = Lock()
        self._snapshot: dict[str, Any] = {
            "schema_version": "1.0",
            "source": "unavailable",
            "sequence": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "system": {
                "valid": False,
                "age_ms": 0,
                "message": "尚未连接统一状态服务",
            },
        }
        self._received_at = 0.0
        self._error = "尚未连接统一状态服务"
        self._task: asyncio.Task[None] | None = None

    @staticmethod
    def _fetch() -> dict[str, Any]:
        with urlopen(UPSTREAM_URL, timeout=0.8) as response:
            return json.load(response)

    async def run(self) -> None:
        while True:
            try:
                snapshot = await asyncio.to_thread(self._fetch)
                with self._lock:
                    self._snapshot = snapshot
                    self._received_at = monotonic()
                    self._error = ""
            except Exception as error:
                with self._lock:
                    self._error = str(error)
            await asyncio.sleep(0.2)

    async def start(self) -> None:
        self._task = asyncio.create_task(self.run())

    async def close(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    def get(self) -> dict[str, Any]:
        with self._lock:
            snapshot = json.loads(json.dumps(self._snapshot))
            received_at = self._received_at
            error = self._error
        gateway_age = int((monotonic() - received_at) * 1000) if received_at else None
        upstream_system = snapshot.get("system") or {}
        upstream_age = int(upstream_system.get("age_ms", 999999999))
        gateway_valid = bool(
            gateway_age is not None
            and gateway_age <= 1500
            and upstream_system.get("valid")
            and upstream_age <= 1500
        )
        if not gateway_valid and not error:
            error = f"共同数据已经过期：{upstream_age}ms"
        snapshot["gateway"] = {
            "valid": gateway_valid,
            "age_ms": gateway_age,
            "upstream": UPSTREAM_URL,
            "message": "上游连接正常" if gateway_valid else f"上游不可用：{error}",
        }
        if not gateway_valid and "system" in snapshot:
            snapshot["system"]["valid"] = False
            snapshot["system"]["message"] = snapshot["gateway"]["message"]
        return snapshot


cache = UpstreamCache()


@asynccontextmanager
async def lifespan(_: FastAPI):
    await cache.start()
    try:
        yield
    finally:
        await cache.close()
        stop_pry_service()


app = FastAPI(title="Platform B Gateway", lifespan=lifespan)


class MotionRequest(BaseModel):
    confirmed_clear: bool = False
    confirm_text: str = ""


class ClampMotionRequest(MotionRequest):
    clamp_mm: float = 5.0
    speed_mm_s: float = 10.0
    capture_id: str = ""


class PryMotionRequest(MotionRequest):
    direction: str = "X_PLUS"
    angle_deg: float = 0.0
    pry_position_mm: float = 100.0
    lever_arm_mm: float = 100.0
    speed_mm_s: float = 40.0


class ControlParamsRequest(BaseModel):
    pry_displacement_mm: float = 100.0
    clamp_displacement_mm: float = 5.0
    speed_mm_s: float = 20.0
    force_limit_n: float = 80.0
    torque_limit_nm: float = 8.0
    hold_seconds: float = 3.0
    pry_direction: str = "X_PLUS"
    pry_angle_deg: float = 45.0


def upstream_json(path: str, method: str = "GET") -> dict[str, Any]:
    request = Request(
        f"{UPSTREAM_URL.rsplit('/api/state', 1)[0]}{path}", method=method
    )
    with urlopen(request, timeout=1.5) as response:
        return json.load(response)


def upstream_bytes(path: str) -> bytes:
    with urlopen(f"{UPSTREAM_URL.rsplit('/api/state', 1)[0]}{path}", timeout=1.5) as response:
        return response.read()


def upstream_mjpeg(path: str) -> StreamingResponse:
    """透传状态服务的预览流，不在网关积压逐帧请求。"""
    try:
        response = urlopen(
            f"{UPSTREAM_URL.rsplit('/api/state', 1)[0]}{path}", timeout=3.0
        )
    except Exception as error:
        raise HTTPException(status_code=503, detail=f"D435 预览流不可用：{error}") from error

    def body():
        try:
            while True:
                # 64 KiB makes several JPEG frames wait in the proxy buffer and
                # visibly turns a 10 fps camera into bursty video.  Small chunks
                # preserve the multipart framing while keeping latency low.
                chunk = response.read(8192)
                if not chunk:
                    return
                yield chunk
        finally:
            response.close()

    return StreamingResponse(
        body(),
        media_type=response.headers.get(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        ),
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


def capture_clamp_plan() -> dict[str, Any]:
    """冻结一次由用户明确请求的稳定视觉结果，供后续执行复用。"""
    global clamp_capture, clamp_capture_at
    plan = clamp_plan()
    center = plan.get("clamp_contact_center_camera_mm")
    width = plan.get("heel_width_mm")
    if not plan.get("valid") or not center or len(center) != 3 or width is None:
        with clamp_capture_lock:
            clamp_capture = None
            clamp_capture_at = 0.0
        plan["captured"] = False
        plan["message"] = plan.get("message") or "当前没有可用于规划的稳定夹持点与宽度"
        return plan
    captured = copy.deepcopy(plan)
    captured["capture_id"] = uuid.uuid4().hex
    captured["captured_at"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    captured["capture_max_age_s"] = CLAMP_CAPTURE_MAX_AGE_S
    captured["captured"] = True
    captured["motion_allowed"] = False
    captured["message"] = "已冻结当前稳定夹持点与宽度；确认执行时将只使用这次规划结果"
    with clamp_capture_lock:
        clamp_capture = captured
        clamp_capture_at = monotonic()
    return copy.deepcopy(captured)


def captured_clamp_plan() -> tuple[dict[str, Any] | None, str | None]:
    global clamp_capture, clamp_capture_at
    with clamp_capture_lock:
        if clamp_capture is None:
            return None, "请先获取夹持点与宽度"
        age_s = monotonic() - clamp_capture_at
        if age_s > CLAMP_CAPTURE_MAX_AGE_S:
            clamp_capture = None
            clamp_capture_at = 0.0
            return None, "已冻结的夹挤规划超过30秒，必须重新获取夹持点与宽度"
        result = copy.deepcopy(clamp_capture)
    result["capture_age_s"] = round(age_s, 1)
    return result, None


def _set_workflow(**values: Any) -> None:
    with workflow_lock:
        workflow_state.update(values)


def workflow_snapshot() -> dict[str, Any]:
    with workflow_lock:
        return dict(workflow_state)


def _pry_service_request(path: str, method: str = "GET") -> dict[str, Any]:
    from urllib.request import Request

    request = Request(f"{PRY_SERVICE_URL}{path}", method=method)
    with urlopen(request, timeout=3.0) as response:
        return json.load(response)


def ensure_pry_service() -> None:
    global pry_service_process
    try:
        _pry_service_request("/health")
        return
    except Exception:
        pass
    if pry_service_process is not None and pry_service_process.poll() is None:
        raise RuntimeError("撬拨视觉子服务正在启动或不可用")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT.parent) + os.pathsep + env.get("PYTHONPATH", "")
    port = PRY_SERVICE_URL.rsplit(":", 1)[-1]
    pry_service_process = subprocess.Popen(
        [sys.executable, str(ROOT / "pry_vision_service.py"), "--host", "127.0.0.1", "--port", port],
        cwd=ROOT.parent,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    deadline = monotonic() + 8.0
    while monotonic() < deadline:
        try:
            _pry_service_request("/health")
            return
        except Exception:
            time.sleep(0.1)
    raise RuntimeError("撬拨视觉子服务启动超时")


def stop_pry_service() -> None:
    global pry_service_process
    try:
        _pry_service_request("/stop", method="POST")
    except Exception:
        pass
    if pry_service_process is not None and pry_service_process.poll() is None:
        pry_service_process.terminate()
    pry_service_process = None


def require_motion_confirmation(request: MotionRequest) -> None:
    if not request.confirmed_clear:
        raise HTTPException(status_code=400, detail="未确认现场无人、无障碍且实体急停可用")
    if request.confirm_text.strip() != "确认运动":
        raise HTTPException(status_code=400, detail="请输入“确认运动”后再提交机械臂运动")


def run_workflow(command: list[str], branch: str, label: str) -> None:
    env = os.environ.copy()
    env["FR5_PLATFORM_ROOT"] = str(ROOT.parent)
    env["PYTHONPATH"] = str(ROOT.parent) + os.pathsep + env.get("PYTHONPATH", "")
    try:
        completed = subprocess.run(
            command,
            cwd=ROOT.parent,
            env=env,
            text=True,
            capture_output=True,
            timeout=360,
            check=False,
        )
        output = "\n".join(part.strip() for part in (completed.stdout, completed.stderr) if part.strip())
        _set_workflow(
            stage="completed" if completed.returncode == 0 else "failed",
            message=output.splitlines()[-1] if output else ("流程完成" if completed.returncode == 0 else "流程失败"),
            returncode=completed.returncode,
        )
    except Exception as error:
        _set_workflow(stage="failed", message=str(error), returncode=-1)
    finally:
        _set_workflow(active=False)


def launch_workflow(command: list[str], branch: str, label: str) -> dict[str, Any]:
    with workflow_lock:
        if workflow_state["active"]:
            raise HTTPException(status_code=409, detail="已有一条真实工作流正在执行")
        # Set this before starting the worker thread.  The old implementation
        # let the thread set it asynchronously, so the HTTP response could
        # still say idle and a second click could submit another workflow.
        workflow_state.update(
            {
                "active": True,
                "branch": branch,
                "stage": label,
                "message": "已提交到现有脚本",
                "returncode": None,
            }
        )
    thread = threading.Thread(target=run_workflow, args=(command, branch, label), daemon=True)
    thread.start()
    return workflow_snapshot()


@app.get("/")
def index() -> FileResponse:
    return FileResponse(ROOT / "control.html")


@app.get("/health")
def health() -> dict[str, Any]:
    snapshot = cache.get()
    return {
        "ok": snapshot["gateway"]["valid"],
        "message": snapshot["gateway"]["message"],
        "source": snapshot.get("source"),
        "sequence": snapshot.get("sequence"),
    }


@app.get("/api/state")
def state() -> dict[str, Any]:
    return cache.get()


@app.get("/api/d435/annotated.mjpg")
def d435_annotated_mjpeg() -> StreamingResponse:
    """带足跟识别标注的 D435 视频流；图像为正立方向。"""
    return upstream_mjpeg("/api/d435/annotated.mjpg")


@app.get("/api/platform-a/clamp/plan")
def clamp_plan() -> dict[str, Any]:
    try:
        return upstream_json("/api/platform-a/clamp/plan")
    except Exception as error:
        return {"valid": False, "motion_allowed": False, "message": f"共同视觉服务不可用：{error}"}


@app.post("/api/workflow/clamp/vision/start")
def start_clamp_vision() -> dict[str, Any]:
    return {
        "branch": "clamp",
        "running": True,
        "algorithm": "state_service.D435Monitor (YOLO + depth-guided heel fallback + HorizontalDiameterEstimator)",
        "message": "夹挤视觉已接入真实 D435；请保持足跟稳定后获取夹持点与宽度",
    }


@app.post("/api/workflow/clamp/plan")
def capture_clamp_plan_route() -> dict[str, Any]:
    return capture_clamp_plan()


@app.post("/api/workflow/pry/vision/start")
def start_pry_vision() -> dict[str, Any]:
    # 撬拨必须和夹挤使用同一份 D435 视觉结果。此前这里会启动一个只接受
    # YOLO 掩膜的独立进程，因而会在夹挤的深度引导回退已经有效时误报“未识别”。
    # 关闭可能遗留的旧子服务，避免它继续占用相机订阅或误导诊断。
    stop_pry_service()
    plan = pry_plan()
    return {
        "branch": "pry",
        "running": True,
        "algorithm": "shared state_service.D435Monitor (same result as clamp)",
        "message": plan.get("message") if plan.get("valid") else "撬拨已接入夹挤共用视觉；等待有效足跟结果",
        "vision_source": "shared_clamp_vision",
    }


@app.post("/api/workflow/pry/vision/stop")
def stop_pry_vision_route() -> dict[str, Any]:
    stop_pry_service()
    return {"branch": "pry", "running": False, "message": "撬拨视觉预览已停止"}


@app.get("/api/workflow/pry/plan")
def pry_plan() -> dict[str, Any]:
    """Return the same live D435 result used by the clamp workflow.

    A pry move still receives a fresh camera-to-base transform at the current
    flange pose, but the 2D target, width and surface gap are computed by the
    exact same monitor/estimator as the clamp page.
    """
    try:
        plan = upstream_json("/api/platform-a/clamp/plan")
    except Exception as error:
        return {
            "valid": False,
            "motion_allowed": False,
            "message": f"撬拨共用视觉不可用：{error}",
            "vision_source": "shared_clamp_vision",
        }

    result = copy.deepcopy(plan)
    result["branch"] = "pry"
    result["vision_source"] = "shared_clamp_vision"
    result["algorithm"] = "state_service.D435Monitor (same as clamp)"
    if result.get("valid"):
        result["message"] = "已复用夹挤视觉的夹持点、宽度和表面间距；可执行撬拨规划"
    else:
        detail = result.get("message") or "当前没有有效足跟结果"
        result["message"] = f"撬拨共用视觉暂不可用：{detail}"
    return result


@app.get("/api/workflow/pry/preview.png")
def pry_preview() -> Response:
    try:
        frame = upstream_bytes("/api/platform-a/clamp/preview.png")
        return Response(
            frame,
            media_type="image/png",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )
    except Exception as error:
        raise HTTPException(status_code=503, detail=f"撬拨共用视觉尚未产生画面：{error}") from error


@app.get("/api/workflow/status")
def workflow_status() -> dict[str, Any]:
    return workflow_snapshot()


@app.post("/api/workflow/clamp/move")
def move_clamp(
    confirmed_clear: bool = False,
    confirm_text: str = "",
    capture_id: str = "",
    clamp_mm: float = 5.0,
    speed_mm_s: float = 10.0,
    dry_run: bool = False,
) -> dict[str, Any]:
    # 使用查询参数而非请求体：规避本项目 Pydantic 请求体解析在特定模型上的已知问题。
    if not confirmed_clear:
        raise HTTPException(status_code=400, detail="未确认现场无人、无障碍且实体急停可用")
    if confirm_text.strip() != "确认运动":
        raise HTTPException(status_code=400, detail="请输入“确认运动”后再提交机械臂运动")
    if not 0.0 <= clamp_mm <= 40.0 or not 0.1 <= speed_mm_s <= 20.0:
        raise HTTPException(status_code=400, detail="夹挤位移必须在0～40mm，速度必须在0.1～20mm/s")
    plan, capture_error = captured_clamp_plan()
    if plan is None:
        raise HTTPException(status_code=409, detail=capture_error or "夹挤规划不可用")
    if capture_id and capture_id != plan.get("capture_id"):
        raise HTTPException(status_code=409, detail="页面中的夹挤规划已经不是当前冻结结果，请重新获取")
    center = plan.get("clamp_contact_center_camera_mm")
    width = plan.get("heel_width_mm")
    if not center or width is None:
        raise HTTPException(status_code=409, detail="夹挤结果缺少相机夹持中心或宽度")
    command = [
        sys.executable, str(ROOT.parent / "scripts/clamp_acquire_and_move.py"),
        "--clamp-mm", f"{clamp_mm:.3f}",
        "--speed-mm-s", f"{speed_mm_s:.3f}",
        f"--center-camera-mm={','.join(f'{float(v):.5f}' for v in center)}",
        "--width-mm", f"{float(width):.5f}",
    ]
    if dry_run:
        command.append("--dry-run")
    return launch_workflow(command, "clamp", "夹挤：移动到夹持点并执行夹挤")


@app.post("/api/workflow/pry/move")
def move_pry(
    confirmed_clear: bool = False,
    confirm_text: str = "",
    direction: str = "X_PLUS",
    angle_deg: float = 0.0,
    pry_position_mm: float = 100.0,
    lever_arm_mm: float = 100.0,
    speed_mm_s: float = 40.0,
    dry_run: bool = False,
) -> dict[str, Any]:
    if not confirmed_clear:
        raise HTTPException(status_code=400, detail="未确认现场无人、无障碍且实体急停可用")
    if confirm_text.strip() != "确认运动":
        raise HTTPException(status_code=400, detail="请输入“确认运动”后再提交机械臂运动")
    if direction not in {"X_PLUS", "X_MINUS", "Y_PLUS", "Y_MINUS"}:
        raise HTTPException(status_code=400, detail="撬拨方向无效")
    if not 0.0 <= angle_deg <= 89.0 or not 0.0 < pry_position_mm <= 150.0:
        raise HTTPException(status_code=400, detail="撬拨角度必须在0～89度，夹持位置必须在0～150mm")
    plan = pry_plan()
    if not plan.get("valid") or not plan.get("clamp_contact_center_camera_mm"):
        raise HTTPException(status_code=409, detail=plan.get("message") or "撬拨视觉结果无效")
    center = plan["clamp_contact_center_camera_mm"]
    gap = float(plan.get("surface_to_upper_midpoint_gap_mm", 0.0))
    if gap <= 0.0:
        raise HTTPException(status_code=409, detail="撬拨结果缺少足跟表面间距")
    command = [
        sys.executable, str(ROOT.parent / "scripts/pry_move_to_clamp.py"),
        f"--center-camera-mm={','.join(f'{float(v):.5f}' for v in center)}",
        "--surface-gap-mm", f"{gap:.5f}",
        "--pry-position-mm", f"{pry_position_mm:.5f}",
        "--pry-direction", direction,
        "--pry-angle-deg", f"{angle_deg:.5f}",
        "--pry-lever-arm-mm", f"{lever_arm_mm:.5f}",
        "--speed-mm-s", f"{speed_mm_s:.5f}",
        "--confirmed-clear", "--experimental-first-six", "--close-after",
    ]
    if dry_run:
        command.append("--dry-run")
    return launch_workflow(command, "pry", "撬拨：移动到夹持点并执行撬拨")


# ----------------------------- 网页直控真机 FR5 -----------------------------
# 这些端点直接通过 Fairino SDK 向 192.168.58.2 下发指令，与 tk端 RealDeviceAdapter
# 同一路径。除“设置零点”仅写文件外，其余均要求 confirm_text == "确认运动"。

@app.get("/api/robot/params")
def robot_params_get() -> dict[str, Any]:
    return {"ok": True, **load_params()}


@app.post("/api/robot/params")
def robot_params_set(request: ControlParamsRequest) -> dict[str, Any]:
    try:
        merged = save_params(request.model_dump())
    except Exception as error:
        raise HTTPException(status_code=400, detail=f"参数保存失败：{error}") from error
    return {"ok": True, "message": "参数已保存（仅存储于网关，运动指令仍由对应流程读取）", **merged}


@app.post("/api/robot/{action}")
def robot_action(action: str, ack_text: str = "") -> dict[str, Any]:
    """暂停/继续/回零/设置零点/急停/急停复位，全部直连真机。"""
    try:
        result = robot_dispatch(action, ack_text)
    except RobotControlError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    except Exception as error:
        raise HTTPException(status_code=503, detail=f"真机指令下发失败：{error}") from error
    return result


@app.post("/api/robot/action/pause")
def robot_pause(ack_text: str = "") -> dict[str, Any]:
    return robot_action("pause", ack_text)


@app.post("/api/robot/action/resume")
def robot_resume(ack_text: str = "") -> dict[str, Any]:
    return robot_action("resume", ack_text)


@app.post("/api/robot/action/home")
def robot_home(ack_text: str = "") -> dict[str, Any]:
    return robot_action("home", ack_text)


@app.post("/api/robot/action/set-zero")
def robot_set_zero(ack_text: str = "") -> dict[str, Any]:
    return robot_action("set-zero", ack_text)


@app.post("/api/robot/action/emergency-stop")
def robot_emergency_stop(ack_text: str = "") -> dict[str, Any]:
    return robot_action("emergency-stop", ack_text)


@app.post("/api/robot/action/emergency-reset")
def robot_emergency_reset(ack_text: str = "") -> dict[str, Any]:
    return robot_action("emergency-reset", ack_text)


@app.post("/api/robot/action/set-gripper-opening")
def robot_set_gripper_opening(opening_mm: float, ack_text: str = "") -> dict[str, Any]:
    """直接设置 AG95 夹爪目标开度（0~95 mm）。"""
    try:
        result = set_gripper_opening(opening_mm, ack_text)
    except RobotControlError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    except Exception as error:
        raise HTTPException(status_code=503, detail=f"夹爪开度设置失败：{error}") from error
    return result


@app.websocket("/ws")
async def websocket_state(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(cache.get())
            await asyncio.sleep(0.25)
    except (WebSocketDisconnect, RuntimeError):
        return


app.mount("/assets", StaticFiles(directory=ROOT), name="assets")


if __name__ == "__main__":
    print("平台2已启动：http://127.0.0.1:8080")
    uvicorn.run(app, host="127.0.0.1", port=8080, log_level="info")
