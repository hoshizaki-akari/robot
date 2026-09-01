#!/usr/bin/env python3
"""FastAPI application for the clean FR5 traction workstation."""

from __future__ import annotations

import asyncio
import csv
import io
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

try:
    from .backend.ros_bridge import RosBridge, RosBridgeError
except ImportError:
    from backend.ros_bridge import RosBridge, RosBridgeError


ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
bridge = RosBridge()


@asynccontextmanager
async def lifespan(_: FastAPI):
    bridge.start()
    try:
        yield
    finally:
        bridge.stop()


app = FastAPI(title="FR5 Traction Workstation", lifespan=lifespan)
app.mount("/assets", StaticFiles(directory=STATIC), name="assets")


class TargetRequest(BaseModel):
    target_force_n: float = Field(ge=1.0, le=20.0)


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
    return bridge.snapshot()["history"]


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
    snapshot = bridge.snapshot()
    summaries = snapshot["history"].get("summaries", [])
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(
        ["session_id", "start_time", "end_time", "target_force_n", "average_force_n", "max_force_n", "final_state", "stop_reason", "record_path"]
    )
    for item in summaries:
        writer.writerow(
            [
                item.get("session_id", ""),
                item.get("start_time", ""),
                item.get("end_time", ""),
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


@app.websocket("/ws")
async def websocket_state(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(bridge.snapshot())
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        return
