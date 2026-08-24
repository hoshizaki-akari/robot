from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager
from threading import Lock
from time import sleep
from typing import Any

import cv2
import numpy as np
from fastapi import FastAPI, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

from .schema import public_snapshot, unavailable_snapshot
from .sources import build_source
from platform_a.clamp_planner import build_clamp_plan


MAX_SNAPSHOT_AGE_MS = 1500
REAL_SAMPLE_TIMEOUT_S = 5.0


class StateStore:
    def __init__(self, source_name: str) -> None:
        self.source = build_source(source_name)
        self._snapshot = unavailable_snapshot(source_name, "状态服务正在启动")
        self._lock = Lock()
        self._task: asyncio.Task[None] | None = None

    def get(self) -> dict[str, Any]:
        with self._lock:
            snapshot = public_snapshot(self._snapshot)
        system_age = int(snapshot.get("system", {}).get("age_ms", 0))
        if system_age > MAX_SNAPSHOT_AGE_MS:
            message = f"实时数据已停止更新：{system_age}ms"
            for name in ("system", "fr5", "kwr75d", "ag95", "d435"):
                device = snapshot.get(name)
                if not isinstance(device, dict):
                    continue
                device["valid"] = False
                if "connected" in device:
                    device["connected"] = False
                device["message"] = message
        return snapshot

    async def run(self) -> None:
        while True:
            try:
                sample = asyncio.to_thread(self.source.sample)
                if self.source.name == "real":
                    snapshot = await asyncio.wait_for(
                        sample, timeout=REAL_SAMPLE_TIMEOUT_S
                    )
                else:
                    snapshot = await sample
            except TimeoutError:
                snapshot = unavailable_snapshot(
                    getattr(self.source, "name", "unknown"),
                    "真实设备读取超过5秒，状态服务即将自动重启",
                )
                with self._lock:
                    self._snapshot = snapshot
                await asyncio.sleep(0.2)
                # A vendor SDK call may be blocked in a worker thread and cannot
                # be cancelled safely. Let systemd restart the whole process.
                os._exit(70)
            except Exception as error:
                snapshot = unavailable_snapshot(
                    getattr(self.source, "name", "unknown"),
                    f"数据源异常：{error}",
                )
            with self._lock:
                self._snapshot = snapshot
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
        await asyncio.to_thread(self.source.close)


SOURCE_NAME = os.environ.get("FR5_STATE_SOURCE", "replay").strip().lower()
store = StateStore(SOURCE_NAME)


@asynccontextmanager
async def lifespan(_: FastAPI):
    await store.start()
    try:
        yield
    finally:
        await store.close()


app = FastAPI(title="FR5 Platform Read-only State Service", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:8080", "http://localhost:8080"],
    allow_credentials=False,
    allow_methods=["GET"],
    allow_headers=["*"],
)


@app.get("/health")
def health() -> dict[str, Any]:
    snapshot = store.get()
    return {
        "ok": bool(snapshot["system"]["valid"]),
        "source": snapshot["source"],
        "sequence": snapshot["sequence"],
        "message": snapshot["system"]["message"],
    }


@app.get("/api/state")
def state() -> dict[str, Any]:
    return store.get()


@app.get("/api/d435/color.png")
def d435_color() -> Response:
    monitor = getattr(store.source, "d435", None)
    image = monitor.color_png() if monitor is not None else None
    if image is None:
        return Response(status_code=503)
    return Response(
        content=image,
        media_type="image/png",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.get("/api/d435/color.mjpg")
def d435_color_mjpeg() -> StreamingResponse:
    """低开销的真实 D435 原始预览流；不参与视觉计算。"""
    monitor = getattr(store.source, "d435", None)
    if monitor is None:
        return StreamingResponse(iter(()), status_code=503)

    def frames():
        last_sequence = -1
        while True:
            snapshot = monitor.color_jpeg_snapshot()
            if snapshot is not None:
                sequence, image = snapshot
                if sequence != last_sequence:
                    last_sequence = sequence
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Cache-Control: no-store\r\n\r\n"
                        + image
                        + b"\r\n"
                    )
            sleep(0.02)

    return StreamingResponse(
        frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


def _png_bytes_to_jpeg(png_bytes: bytes, quality: int = 85) -> bytes | None:
    arr = np.frombuffer(png_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return None
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else None


@app.get("/api/d435/annotated.mjpg")
def d435_annotated_mjpeg() -> StreamingResponse:
    """带 YOLO 足跟识别标注的 D435 预览流；图像已旋转回正立方向。"""
    monitor = getattr(store.source, "d435", None)
    if monitor is None:
        return StreamingResponse(iter(()), status_code=503)

    def frames():
        last_signature = -1
        while True:
            png = monitor.annotated_png()
            if png is not None:
                # 用长度与前 8 字节构造简单签名，避免同一帧重复编码。
                signature = len(png) ^ int.from_bytes(png[:8], "little", signed=False)
                if signature != last_signature:
                    last_signature = signature
                    jpeg = _png_bytes_to_jpeg(png)
                    if jpeg is not None:
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Cache-Control: no-store\r\n\r\n"
                            + jpeg
                            + b"\r\n"
                        )
            sleep(0.05)

    return StreamingResponse(
        frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.get("/api/platform-a/clamp/preview.png")
def clamp_preview() -> Response:
    monitor = getattr(store.source, "d435", None)
    image = monitor.annotated_png() if monitor is not None else None
    if image is None:
        return Response(status_code=503)
    return Response(
        content=image,
        media_type="image/png",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.get("/api/platform-a/clamp/plan")
def clamp_plan() -> dict[str, Any]:
    monitor = getattr(store.source, "d435", None)
    if monitor is None:
        return {"valid": False, "motion_allowed": False, "message": "当前没有真实相机"}
    vision = monitor.vision_snapshot()
    snapshot = store.get()
    return build_clamp_plan(vision, snapshot.get("fr5") or {})


@app.websocket("/ws")
async def websocket_state(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(store.get())
            await asyncio.sleep(0.25)
    except (WebSocketDisconnect, RuntimeError):
        return
