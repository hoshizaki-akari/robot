#!/usr/bin/env python3
"""Isolated read-only HTTP wrapper for the existing pry vision worker."""
from __future__ import annotations

import argparse
from contextlib import asynccontextmanager
from typing import Any

import uvicorn
from fastapi import FastAPI, HTTPException, Response

from platform_a.pry_buckle_vision import PryBuckleVisionWorker


worker = PryBuckleVisionWorker()


@asynccontextmanager
async def lifespan(_: FastAPI):
    yield
    worker.stop()


app = FastAPI(title="Isolated Pry Vision Service", lifespan=lifespan)


@app.get("/health")
def health() -> dict[str, Any]:
    result = worker.result
    return {"ok": True, "running": worker._thread is not None and worker._thread.is_alive(), "valid": bool(result.get("valid")), "message": result.get("message", "")}


@app.post("/start")
def start() -> dict[str, Any]:
    worker.start()
    return {"running": True, "message": "撬拨视觉线程已启动"}


@app.post("/stop")
def stop() -> dict[str, Any]:
    worker.stop()
    return {"running": False, "message": "撬拨视觉线程已停止"}


@app.get("/plan")
def plan() -> dict[str, Any]:
    return worker.result


@app.get("/preview")
def preview() -> Response:
    frame = worker.frame_png
    if not frame:
        raise HTTPException(status_code=503, detail="撬拨视觉尚未产生画面")
    media_type = "image/png" if frame.startswith(b"\x89PNG") else "image/jpeg"
    return Response(frame, media_type=media_type, headers={"Cache-Control": "no-store"})


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


