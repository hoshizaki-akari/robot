"""FastAPI entry point for the standalone KWR75D demo."""

from __future__ import annotations

import asyncio
import json
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from force_demo import CsvRecorder, EngineConfig, ForceTensionEngine
from force_demo.sources import SourceManager

ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "config.json"
STATIC = ROOT / "static"


def load_config() -> dict[str, object]:
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    values = load_config()
    debug_root = Path(os.environ.get("FORCE_DEMO_DEBUG_DIR", ROOT / "debug" / "sessions"))
    recorder = CsvRecorder(debug_root)
    engine = ForceTensionEngine(EngineConfig.from_mapping(values), recorder=recorder)
    sources = SourceManager(engine, values)
    app.state.engine = engine
    app.state.sources = sources
    app.state.recorder = recorder
    if os.environ.get("FORCE_DEMO_DISABLE_SOURCES") != "1":
        sources.start()
    try:
        yield
    finally:
        if os.environ.get("FORCE_DEMO_DISABLE_SOURCES") != "1":
            sources.stop()
        engine.close()


app = FastAPI(
    title="KWR75D 绳带松紧检测 Demo",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/api/docs",
    redoc_url=None,
)
app.mount("/assets", StaticFiles(directory=STATIC), name="assets")


def engine_from(request: Request) -> ForceTensionEngine:
    return request.app.state.engine


@app.get("/", include_in_schema=False)
async def index() -> FileResponse:
    return FileResponse(STATIC / "index.html")


@app.get("/api/health")
async def health(request: Request) -> dict[str, object]:
    state = engine_from(request).snapshot()
    return {"ok": True, "sensor_connected": state["connected"], "phase": state["phase"]}


@app.get("/api/state")
async def state(request: Request, history: bool = False) -> dict[str, object]:
    return engine_from(request).snapshot(include_history=history)


@app.post("/api/baseline")
async def baseline(request: Request) -> dict[str, object]:
    engine = engine_from(request)
    success, message = engine.begin_baseline()
    if not success:
        raise HTTPException(status_code=409, detail=message)
    return {"success": True, "message": message, "state": engine.snapshot()}


@app.delete("/api/baseline")
async def clear_baseline(request: Request) -> dict[str, object]:
    engine = engine_from(request)
    engine.clear_baseline()
    return {"success": True, "state": engine.snapshot()}


@app.post("/api/direction/reverse")
async def reverse_direction(request: Request) -> dict[str, object]:
    engine = engine_from(request)
    sign = engine.reverse_increase_direction()
    return {"success": True, "increase_direction_sign": sign, "state": engine.snapshot()}


@app.get("/api/export/latest")
async def export_latest(request: Request) -> FileResponse:
    request.app.state.recorder.flush()
    path: Path = request.app.state.recorder.csv_path
    if not path.exists():
        raise HTTPException(status_code=404, detail="本次运行还没有 CSV")
    return FileResponse(path, media_type="text/csv", filename=path.name)


@app.websocket("/ws")
async def websocket_state(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(websocket.app.state.engine.snapshot())
            await asyncio.sleep(0.05)
    except (WebSocketDisconnect, RuntimeError):
        return
