#!/usr/bin/env python3
"""FastAPI application for the clean FR5 traction workstation."""

from __future__ import annotations

import asyncio
import csv
import io
import json
import tempfile
import threading
import time
import zipfile
from contextlib import asynccontextmanager
from pathlib import Path
from urllib.parse import unquote

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from starlette.background import BackgroundTask

try:
    from .backend.ros_bridge import RosBridge, RosBridgeError
except ImportError:
    from backend.ros_bridge import RosBridge, RosBridgeError


ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
SESSION_ROOT = (ROOT / "debug" / "traction_sessions").resolve()
OPERATIONS_PATH = SESSION_ROOT / "operations.csv"
OPERATION_LOCK = threading.Lock()
bridge = RosBridge()

OPERATION_NAMES = {
    "/api/traction/prepare": "初始校准",
    "/api/traction/calibrate-direction": "方向确定",
    "/api/traction/target": "设置目标力",
    "/api/traction/settings": "设置最大行程",
    "/api/traction/start": "开始牵引",
    "/api/traction/stop": "结束牵引",
    "/api/traction/emergency-stop": "急停",
    "/api/traction/reset-fault": "故障复位",
    "/api/traction/set-zero": "设置零点",
    "/api/traction/return-zero": "回零",
}


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
    bridge.start()
    try:
        yield
    finally:
        bridge.stop()


app = FastAPI(title="FR5 Traction Workstation", lifespan=lifespan)
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
    target_force_n: float = Field(ge=1.0, le=20.0)


class MotionSettingsRequest(BaseModel):
    max_travel_mm: float = Field(ge=50.0, le=500.0)


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC / "090105.html")


@app.get("/api/health")
def health() -> dict:
    snapshot = bridge.snapshot()
    return {
        "ok": snapshot["connected"],
        "fr5_connected": snapshot["fr5"]["connected"],
        "traction_connected": snapshot["traction"]["connected"],
        "state": snapshot["traction"].get("state_name", "INITIALIZING"),
    }


@app.get("/api/state")
def state() -> dict:
    return bridge.snapshot()


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


@app.get("/api/traction/settings")
def motion_settings() -> dict:
    try:
        return bridge.get_motion_settings()
    except RosBridgeError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


def _call(name: str, target_force_n: float | None = None) -> dict:
    try:
        return bridge.call(name, target_force_n)
    except RosBridgeError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


@app.post("/api/traction/prepare")
def prepare() -> dict:
    return _call("prepare")


@app.post("/api/traction/calibrate-direction")
def calibrate_direction() -> dict:
    return _call("calibrate_direction")


@app.post("/api/traction/target")
def set_target(request: TargetRequest) -> dict:
    return _call("set_target_force", request.target_force_n)


@app.post("/api/traction/settings")
def set_motion_settings(request: MotionSettingsRequest) -> dict:
    try:
        return bridge.set_max_travel_mm(request.max_travel_mm)
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
    return _call("emergency_stop")


@app.post("/api/traction/reset-fault")
def reset_fault() -> dict:
    return _call("reset_fault")


@app.post("/api/traction/set-zero")
def set_zero() -> dict:
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
        ["session_id", "start_time", "end_time", "operator", "role", "operation_count", "target_force_n", "average_force_n", "max_force_n", "final_state", "stop_reason", "record_path"]
    )
    for item in summaries:
        writer.writerow(
            [
                item.get("session_id", ""),
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
            "README.txt",
            "traction.csv：从初始校准到结束的完整时序数据，包含目标力、总张力、"
            "Fx/Fy/Fz、原始/滤波/锁定方向、方向状态、末端位置、速度和停止原因。\n"
            "operations.csv：该次记录时间范围内的页面操作、操作者、角色和执行结果。\n"
            "summary.json：本次牵引的开始/结束时间、目标力、平均力、最大力和最终状态。\n",
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
            await websocket.send_json(bridge.snapshot())
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        return
