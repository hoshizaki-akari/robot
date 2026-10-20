#!/usr/bin/env python3
"""FastAPI application for the isolated AUBO i5 traction workstation."""

from __future__ import annotations

import asyncio
import csv
import io
import json
import os
import re
import signal
import tempfile
import threading
import time
import zipfile
from contextlib import asynccontextmanager, suppress
from pathlib import Path
from urllib.parse import unquote

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from starlette.background import BackgroundTask

try:
    from .backend.ros_bridge import RosBridge, RosBridgeError
    from .backend.runtime_recorder import RuntimeRecorder
except ImportError:
    from backend.ros_bridge import RosBridge, RosBridgeError
    from backend.runtime_recorder import RuntimeRecorder


ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
SESSION_ROOT = (ROOT / "debug" / "traction_sessions").resolve()
OPERATIONS_PATH = SESSION_ROOT / "operations.csv"
OPERATION_LOCK = threading.Lock()
SETTINGS_PATH = Path(
    os.environ.get(
        "AUBO_WORKSTATION_SETTINGS_PATH",
        Path.home() / ".local" / "state" / "aubo-i5-traction" / "settings.json",
    )
).expanduser()
SETTINGS_LOCK = threading.Lock()
DEFAULT_SETTINGS = {"traction_force_limit_n": 100.0}
bridge = RosBridge()
runtime_recorder = RuntimeRecorder(ROOT / "debug" / "runtime_feedback")
runtime_recording_status = {"available": False, "error": "not started"}
MOTION_ENABLED = os.environ.get("AUBO_TRACTION_ENABLED", "false").lower() in {
    "1", "true", "yes", "on"
}

OPERATION_NAMES = {
    "/api/traction/prepare": "初始校准",
    "/api/traction/calibrate-direction": "方向确定",
    "/api/traction/target": "设置目标力",
    "/api/traction/mode": "切换工作模式",
    "/api/traction/start": "开始牵引",
    "/api/traction/stop": "结束牵引",
    "/api/traction/emergency-stop": "急停",
    "/api/traction/emergency-recover": "急停恢复",
    "/api/traction/reset-fault": "故障复位",
    "/api/traction/set-zero": "设置当前位置为零位",
    "/api/traction/return-zero": "回零",
    "/api/settings": "保存参数",
    "/api/system/shutdown": "关闭程序",
}


def _read_settings() -> dict[str, float]:
    with SETTINGS_LOCK:
        try:
            raw = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
        except (FileNotFoundError, OSError, json.JSONDecodeError, TypeError):
            return dict(DEFAULT_SETTINGS)
    try:
        limit = float(raw.get("traction_force_limit_n", 100.0))
    except (TypeError, ValueError):
        limit = 100.0
    return {"traction_force_limit_n": min(100.0, max(1.0, limit))}


def _write_settings(settings: dict[str, float]) -> None:
    with SETTINGS_LOCK:
        SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
        temporary = SETTINGS_PATH.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(settings, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, SETTINGS_PATH)


def _write_operation(
    action: str,
    operator: str,
    role: str,
    detail: str,
    http_status: int,
    snapshot: dict,
) -> None:
    traction = snapshot.get("traction", {})
    row = [
        f"{time.time():.6f}",
        action,
        operator or "--",
        role or "--",
        detail or "{}",
        "成功" if 200 <= http_status < 300 else "失败",
        http_status,
        traction.get("state_name", "UNKNOWN"),
        traction.get("target_force_n", ""),
        traction.get("actual_force_n", ""),
        traction.get("fault_code", ""),
        traction.get("stop_reason", ""),
    ]
    with OPERATION_LOCK:
        SESSION_ROOT.mkdir(parents=True, exist_ok=True)
        exists = OPERATIONS_PATH.exists()
        with OPERATIONS_PATH.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            if not exists:
                writer.writerow(
                    [
                        "timestamp",
                        "action",
                        "operator",
                        "role",
                        "detail",
                        "result",
                        "http_status",
                        "traction_state",
                        "target_force_n",
                        "actual_force_n",
                        "fault_code",
                        "stop_reason",
                    ]
                )
            writer.writerow(row)


def _read_operations() -> list[dict[str, str]]:
    with OPERATION_LOCK:
        if not OPERATIONS_PATH.exists():
            return []
        with OPERATIONS_PATH.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))


def _stamp_value(value: dict | None) -> float:
    if not value:
        return 0.0
    return float(value.get("sec", 0)) + float(value.get("nanosec", 0)) * 1e-9


def _operations_in_session(summary: dict, operations: list[dict]) -> list[dict]:
    started = _stamp_value(summary.get("start_time"))
    ended = _stamp_value(summary.get("end_time"))
    return [
        row
        for row in operations
        # A stop request is written after the ROS service has returned, while
        # the manager finalizes the session just before that response. Keep a
        # small end tolerance so the operator's final click stays with the run.
        if started <= float(row.get("timestamp", 0.0)) <= ended + 2.0
    ]


@asynccontextmanager
async def lifespan(_: FastAPI):
    expected_domain = os.environ.get("AUBO_ROS_DOMAIN_ID", "42")
    if os.environ.get("ROS_DOMAIN_ID") != expected_domain:
        raise RuntimeError(
            "AUBO workstation requires its isolated ROS_DOMAIN_ID; "
            "start it with run_workstation.sh"
        )
    bridge.start()
    recording_task = asyncio.create_task(_record_runtime_feedback())
    try:
        yield
    finally:
        recording_task.cancel()
        with suppress(asyncio.CancelledError):
            await recording_task
        bridge.stop()


async def _record_runtime_feedback() -> None:
    """Persist real feedback independently of browser clicks or open sockets."""
    while True:
        try:
            snapshot = bridge.snapshot()
            snapshot["motion_enabled"] = MOTION_ENABLED
            await asyncio.to_thread(runtime_recorder.append, snapshot)
            runtime_recording_status.update(available=True, error="")
        except (OSError, ValueError, TypeError) as error:
            runtime_recording_status.update(available=False, error=str(error))
        await asyncio.sleep(0.1)


app = FastAPI(title="AUBO i5 Traction Workstation", lifespan=lifespan)
app.mount("/assets", StaticFiles(directory=STATIC), name="assets")


@app.middleware("http")
async def record_operation(request: Request, call_next):
    action = OPERATION_NAMES.get(request.url.path) if request.method == "POST" else None
    if not action or request.url.path == "/api/traction/heartbeat":
        return await call_next(request)
    status_code = 500
    try:
        response = await call_next(request)
        status_code = response.status_code
        return response
    finally:
        operator = unquote(request.headers.get("x-operator", "--"))
        role = unquote(request.headers.get("x-role", "--"))
        detail = unquote(request.headers.get("x-operation-detail", "{}"))
        _write_operation(
            action, operator, role, detail, status_code, bridge.snapshot()
        )


class TargetRequest(BaseModel):
    target_force_n: float = Field(ge=1.0, le=100.0)


class SettingsRequest(BaseModel):
    traction_force_limit_n: float = Field(ge=1.0, le=100.0)


class OperationModeRequest(BaseModel):
    mode: int = Field(ge=0, le=2)


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC / "090105.html")


@app.get("/api/health")
def health() -> dict:
    snapshot = bridge.snapshot()
    return {
        "ok": snapshot["connected"],
        "robot_connected": snapshot["robot"]["connected"],
        "traction_connected": snapshot["traction"]["connected"],
        "state": snapshot["traction"].get("state_name", "INITIALIZING"),
        "motion_enabled": MOTION_ENABLED,
    }


@app.get("/api/state")
def state() -> dict:
    snapshot = bridge.snapshot()
    snapshot["motion_enabled"] = MOTION_ENABLED
    snapshot["runtime_recording"] = dict(runtime_recording_status)
    return snapshot


@app.get("/api/settings")
def get_settings() -> dict:
    return _read_settings()


@app.post("/api/settings")
def save_settings(request: SettingsRequest) -> dict:
    settings = {"traction_force_limit_n": float(request.traction_force_limit_n)}
    _write_settings(settings)
    return settings


@app.get("/api/traction/history")
def history() -> dict:
    result = bridge.snapshot()["history"]
    operations = _read_operations()
    for summary in result.get("summaries", []):
        session_operations = _operations_in_session(summary, operations)
        identified = next(
            (
                row
                for row in session_operations
                if row.get("operator") not in (None, "", "--")
            ),
            None,
        )
        summary["operator"] = identified.get("operator", "--") if identified else "--"
        summary["role"] = identified.get("role", "系统") if identified else "系统"
        summary["operation_count"] = len(session_operations)
    return result


MOTION_SERVICES_UNVERIFIED = {
    "prepare", "calibrate_direction", "start", "set_target_force",
    "set_zero_pose", "return_zero_pose", "reset_fault",
}


def _call(name: str, target_force_n: float | None = None) -> dict:
    if not MOTION_ENABLED and name in MOTION_SERVICES_UNVERIFIED:
        raise HTTPException(
            status_code=409,
            detail="AUBO i5 牵引运动链路未完成真机验证；当前仅允许只读状态和停止请求。",
        )
    try:
        return bridge.call(name, target_force_n)
    except RosBridgeError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


def _terminate_workstation() -> None:
    """Stop the tablet supervisor, or just this web process when run manually."""
    time.sleep(0.5)
    raw_supervisor_pid = os.environ.get("AUBO_WORKSTATION_SUPERVISOR_PID", "")
    supervisor_pid = int(raw_supervisor_pid) if raw_supervisor_pid.isdigit() else 0
    target_pid = supervisor_pid if supervisor_pid > 1 else os.getpid()
    os.kill(target_pid, signal.SIGTERM)


@app.post("/api/system/shutdown")
def shutdown_program() -> dict:
    threading.Thread(target=_terminate_workstation, daemon=True).start()
    return {"success": True, "message": "工作站正在关闭"}


@app.post("/api/traction/prepare")
def prepare() -> dict:
    return _call("prepare")


@app.post("/api/traction/calibrate-direction")
def calibrate_direction() -> dict:
    return _call("calibrate_direction")


@app.post("/api/traction/target")
def set_target(request: TargetRequest) -> dict:
    configured_limit = _read_settings()["traction_force_limit_n"]
    if request.target_force_n > configured_limit:
        raise HTTPException(
            status_code=409,
            detail=f"目标牵引力不能超过当前设定上限 {configured_limit:g} N",
        )
    return _call("set_target_force", request.target_force_n)


@app.post("/api/traction/mode")
def set_operation_mode(request: OperationModeRequest) -> dict:
    if not MOTION_ENABLED:
        raise HTTPException(
            status_code=409,
            detail="AUBO i5 牵引模式尚未完成真机验证，拒绝切换。",
        )
    try:
        return bridge.set_operation_mode(request.mode)
    except RosBridgeError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


@app.post("/api/traction/start")
def start() -> dict:
    return _call("start")


@app.post("/api/traction/stop")
def stop() -> dict:
    return _call("stop")


@app.post("/api/traction/emergency-stop")
def emergency_stop() -> dict:
    # Always try to latch software traction off, even when the V4 stop RPC
    # fails. A successful SDK return is not proof of physical standstill.
    hardware_error = None
    try:
        _call("hardware_emergency_stop")
    except HTTPException as error:
        hardware_error = error
    software_error = None
    try:
        result = _call("emergency_stop")
    except HTTPException as error:
        software_error = error
    if hardware_error is not None:
        raise hardware_error
    if software_error is not None:
        raise software_error
    result["message"] = (
        "AUBO V4 已接受快速停止请求，牵引状态已锁定；请目视确认停稳，并检查实体急停。"
    )
    return result


@app.post("/api/traction/emergency-recover")
def emergency_recover() -> dict:
    raise HTTPException(
        status_code=409,
        detail="AUBO V4 硬件急停恢复尚未验证；请按控制柜/示教器流程人工确认，软件不自动复位。",
    )


@app.post("/api/traction/reset-fault")
def reset_fault() -> dict:
    return _call("reset_fault")


@app.post("/api/traction/set-zero")
def set_zero(request: Request) -> dict:
    role = unquote(request.headers.get("x-role", ""))
    if role != "管理员":
        raise HTTPException(status_code=403, detail="仅管理员可以设置零位")
    state = bridge.snapshot().get("traction", {}).get("state")
    if state not in (1, 2, 5, 8):
        raise HTTPException(status_code=409, detail="请先停止机械臂，再设置当前位置为零位")
    return _call("set_zero_pose")


@app.post("/api/traction/return-zero")
def return_zero() -> dict:
    return _call("return_zero_pose")


@app.post("/api/traction/heartbeat")
def heartbeat() -> dict:
    bridge.heartbeat()
    return {"success": True, "snapshot": bridge.snapshot()}


@app.get("/api/traction/export/latest")
def export_latest() -> Response:
    summaries = history().get("summaries", [])
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(
        ["session_id", "operation_mode", "start_time", "end_time", "operator", "role", "operation_count", "target_force_n", "average_force_n", "max_force_n", "final_state", "stop_reason", "record_path"]
    )
    for item in summaries:
        writer.writerow(
            [
                item.get("session_id", ""),
                item.get("operation_mode", 0),
                item.get("start_time", ""),
                item.get("end_time", ""),
                item.get("operator", ""),
                item.get("role", ""),
                item.get("operation_count", ""),
                item.get("target_force_n", ""),
                item.get("average_force_n", ""),
                item.get("max_force_n", ""),
                item.get("final_state", ""),
                item.get("stop_reason", ""),
                item.get("record_path", ""),
            ]
        )
    return Response(
        content="\ufeff" + output.getvalue(),
        media_type="text/csv; charset=utf-8",
        headers={"Content-Disposition": "attachment; filename=traction_history.csv"},
    )


def _driver_events_in_session(started: float, ended: float) -> str:
    directory = os.environ.get("AUBO_RUNTIME_LOG_DIR")
    debug_root = (ROOT / "debug").resolve()
    candidates = set(debug_root.glob("session_*_workstation/robot.log"))
    if directory:
        candidates.add(Path(directory) / "robot.log")
    events = []
    for candidate in sorted(candidates):
        log_path = candidate.resolve()
        try:
            relative = log_path.relative_to(debug_root)
            selected = []
            with log_path.open(encoding="utf-8", errors="replace") as stream:
                for line in stream:
                    stamp = re.search(r"\[(\d{10}\.\d+)\]", line)
                    if stamp and started - 1.0 <= float(stamp.group(1)) <= ended:
                        selected.append(line)
            if selected:
                events.append(f"[driver log: {relative.as_posix()}]\n")
                events.extend(selected)
        except (ValueError, OSError):
            continue
    return "".join(events)


@app.get("/api/traction/export/session/{session_id}")
def export_session(session_id: str) -> FileResponse:
    if not session_id.startswith("session_") or not session_id[8:].isdigit():
        raise HTTPException(status_code=400, detail="记录编号无效")
    summaries = history().get("summaries", [])
    summary = next(
        (item for item in summaries if item.get("session_id") == session_id), None
    )
    if summary is None:
        raise HTTPException(status_code=404, detail="没有找到这条牵引记录")
    raw_record_path = Path(str(summary.get("record_path", "")))
    record_path = (
        raw_record_path.resolve()
        if raw_record_path.is_absolute()
        else (ROOT / raw_record_path).resolve()
    )
    try:
        record_path.relative_to(SESSION_ROOT)
    except ValueError as error:
        raise HTTPException(status_code=400, detail="记录文件路径无效") from error
    if not record_path.is_file():
        raise HTTPException(status_code=404, detail="详细牵引数据不存在")

    operations = _operations_in_session(summary, _read_operations())
    started = _stamp_value(summary.get("start_time"))
    ended = _stamp_value(summary.get("end_time"))
    runtime_rows = runtime_recorder.rows_between(started, ended + 10.0)
    driver_events = _driver_events_in_session(started, ended + 10.0)
    evidence = {
        "format_version": 1,
        "source": "periodic real ROS bridge snapshots, not reconstructed commands",
        "nominal_sample_period_s": 0.1,
        "session_start_unix_s": started,
        "session_end_unix_s": ended,
        "post_session_context_s": 10.0,
        "sample_count": len(runtime_rows),
        "driver_events_available": bool(driver_events),
        "first_capture_unix_s": runtime_rows[0]["timestamp_wall"] if runtime_rows else None,
        "last_capture_unix_s": runtime_rows[-1]["timestamp_wall"] if runtime_rows else None,
        "recording_status_at_export": dict(runtime_recording_status),
        "interpretation": (
            "Empty/missing feedback is NOT proof of standstill or successful traction. "
            "Inspect robot identity, validity/data ages, diagnostic motion_idle and phase, "
            "joint velocities, force and positions. Post-session context may overlap a later run."
        ),
    }
    operation_output = io.StringIO()
    operation_fields = [
        "timestamp",
        "action",
        "operator",
        "role",
        "detail",
        "result",
        "http_status",
        "traction_state",
        "target_force_n",
        "actual_force_n",
        "fault_code",
        "stop_reason",
    ]
    operation_writer = csv.DictWriter(operation_output, fieldnames=operation_fields)
    operation_writer.writeheader()
    for row in operations:
        operation_writer.writerow({field: row.get(field, "") for field in operation_fields})

    export_directory = ROOT / "debug" / "traction_exports"
    export_directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        prefix=f"{session_id}_", suffix=".zip", dir=export_directory, delete=False
    ) as temporary:
        temporary_path = Path(temporary.name)
    with zipfile.ZipFile(
        temporary_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6
    ) as archive:
        archive.write(record_path, "traction.csv")
        archive.writestr(
            "summary.json", json.dumps(summary, ensure_ascii=False, indent=2)
        )
        archive.writestr("operations.csv", "\ufeff" + operation_output.getvalue())
        archive.writestr(
            "runtime_feedback.jsonl",
            "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in runtime_rows),
        )
        archive.writestr("runtime_evidence.json", json.dumps(evidence, ensure_ascii=False, indent=2))
        archive.writestr("driver_events.log", driver_events)
        archive.writestr(
            "README.txt",
            "traction.csv：从初始校准到结束的完整时序数据，包含目标力、总张力、"
            "Fx/Fy/Fz、原始/滤波/锁定方向、方向状态、末端位置、速度和停止原因。\n"
            "operations.csv：该次记录时间范围内的页面操作、操作者、角色和执行结果。\n"
            "summary.json：本次牵引的开始/结束时间、目标力、平均力、最大力和最终状态。\n"
            "runtime_feedback.jsonl：约10Hz真实运行快照，包括身份验证标志、关节实测值、"
            "数据年龄、驱动阶段、motion_idle、力和末端位置；包含结束后最多10秒上下文。\n"
            "runtime_evidence.json：记录覆盖范围及导出时记录器状态；空记录不是停稳证明。\n"
            "driver_events.log：本次会话期间驱动健康检查失败等事件；空文件表示没有捕获到对应事件。\n"
            "traction.csv中的command/velocity_cmd为命令；ee_v*_mps、ee_w*_radps为实测。\n"
            "历史旧记录可能没有新增反馈文件或列；请勿据此推断其已经验收通过。\n",
        )
    return FileResponse(
        temporary_path,
        media_type="application/zip",
        filename=f"{session_id}_full_log.zip",
        background=BackgroundTask(lambda: temporary_path.unlink(missing_ok=True)),
    )


@app.websocket("/ws")
async def websocket_state(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            snapshot = bridge.snapshot()
            snapshot["motion_enabled"] = MOTION_ENABLED
            snapshot["runtime_recording"] = dict(runtime_recording_status)
            await websocket.send_json(snapshot)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        return
