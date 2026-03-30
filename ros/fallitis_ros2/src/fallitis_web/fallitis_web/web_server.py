from __future__ import annotations

import asyncio
import os
import threading
import time
from pathlib import Path

import cv2
import rclpy
import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse

from .ros_state import RosStateBridge, SharedState


def make_app(bridge: RosStateBridge, state: SharedState) -> FastAPI:
    static_dir = Path(get_package_share_directory("fallitis_web")) / "static"
    app = FastAPI(title="Fall-Itis Web Control")

    def no_cache(response):
        response.headers["Cache-Control"] = "no-store, max-age=0"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
        return response

    def teleop_from_keys(keys: dict[str, bool]) -> tuple[float, float]:
        linear = 0.0
        angular = 0.0
        if keys.get("w"):
            linear += bridge.config.manual_linear_speed_m_s
        if keys.get("s"):
            linear -= bridge.config.manual_linear_speed_m_s
        if keys.get("a"):
            angular += bridge.config.manual_angular_speed_rad_s
        if keys.get("d"):
            angular -= bridge.config.manual_angular_speed_rad_s
        return linear, angular

    def mjpeg_stream(side: str):
        async def iterator():
            while True:
                with state.lock:
                    frame = state.left_jpeg if side == "left" else state.right_jpeg
                if frame is None:
                    await asyncio.sleep(0.1)
                    continue
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
                await asyncio.sleep(0.1)

        return iterator()

    def snapshot_dir() -> Path:
        base = Path("~/fallitis_ros2/snapshots").expanduser()
        base.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        millis = int((time.time() % 1.0) * 1000.0)
        directory = base / f"snapshot-{timestamp}-{millis:03d}"
        directory.mkdir(parents=True, exist_ok=True)
        return directory

    def save_snapshot_frames() -> dict[str, object]:
        with state.lock:
            left = None if state.left_frame is None else state.left_frame.copy()
            right = None if state.right_frame is None else state.right_frame.copy()
            pose = dict(state.pose)

        if left is None and right is None:
            raise HTTPException(status_code=503, detail="No camera frames available yet.")

        directory = snapshot_dir()
        left_path = ""
        right_path = ""
        frames: dict[str, dict[str, int]] = {}

        if left is not None:
            left_path = str(directory / "left.png")
            if not cv2.imwrite(left_path, left):
                raise HTTPException(status_code=500, detail="Failed to write left snapshot.")
            frames["left"] = {"width": int(left.shape[1]), "height": int(left.shape[0])}

        if right is not None:
            right_path = str(directory / "right.png")
            if not cv2.imwrite(right_path, right):
                raise HTTPException(status_code=500, detail="Failed to write right snapshot.")
            frames["right"] = {"width": int(right.shape[1]), "height": int(right.shape[0])}

        payload = {
            "directory": str(directory),
            "left_path": left_path,
            "right_path": right_path,
            "frames": frames,
            "pose": pose,
        }
        summary = str(directory)
        with state.lock:
            state.last_snapshot = summary
        payload["path"] = summary
        return payload

    @app.get("/")
    async def index():
        return no_cache(FileResponse(static_dir / "index.html"))

    @app.get("/assets/{name}")
    async def asset(name: str):
        file_path = static_dir / name
        if not file_path.exists():
            raise HTTPException(status_code=404, detail="Asset not found")
        return no_cache(FileResponse(file_path))

    @app.get("/api/health")
    async def health():
        with state.lock:
            payload = {
                "map_version": state.map_version,
                "has_left": state.left_jpeg is not None,
                "has_right": state.right_jpeg is not None,
                "pose": state.pose,
                "imu": state.imu,
                "lidar_scan": state.lidar_scan,
                "perception_status": state.perception_status,
                "mapping_debug": state.mapping_debug,
                "actuation_status": state.actuation_status,
                "last_snapshot": state.last_snapshot,
            }
        return JSONResponse(payload)

    @app.post("/api/snapshot")
    async def snapshot():
        payload = save_snapshot_frames()
        return JSONResponse(payload)

    @app.post("/api/lop")
    async def lack_of_progress():
        bridge.request_lop()
        return JSONResponse({"ok": True, "message": "LoP requested"})

    @app.post("/api/mode")
    async def set_mode(request: Request):
        payload = await request.json()
        value = payload.get("value")
        if not isinstance(value, str):
            raise HTTPException(status_code=400, detail="Missing string 'value'.")
        try:
            selected = bridge.set_mode(value)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return JSONResponse({"ok": True, "value": selected})

    @app.post("/api/object_recognition")
    async def set_object_recognition(request: Request):
        payload = await request.json()
        value = payload.get("value")
        selected = bridge.set_object_recognition(bool(value))
        return JSONResponse({"ok": True, "value": selected})

    @app.get("/stream/left.mjpg")
    async def left_stream():
        return no_cache(StreamingResponse(
            mjpeg_stream("left"),
            media_type="multipart/x-mixed-replace; boundary=frame",
        ))

    @app.get("/stream/right.mjpg")
    async def right_stream():
        return no_cache(StreamingResponse(
            mjpeg_stream("right"),
            media_type="multipart/x-mixed-replace; boundary=frame",
        ))

    @app.get("/map/latest.png")
    async def map_png():
        with state.lock:
            payload = state.map_png
        if payload is None:
            raise HTTPException(status_code=404, detail="Map not available yet.")
        return no_cache(StreamingResponse(iter([payload]), media_type="image/png"))

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()

        async def push_state():
            last_version = -1
            while True:
                with state.lock:
                    version = state.map_version
                    payload = {
                        "type": "state",
                        "map_version": version,
                        "map_meta": dict(state.map_meta),
                        "pose": dict(state.pose),
                        "imu": dict(state.imu),
                        "lidar_scan": dict(state.lidar_scan),
                        "perception_status": dict(state.perception_status),
                        "mapping_debug": dict(state.mapping_debug),
                        "actuation_status": dict(state.actuation_status),
                        "last_snapshot": state.last_snapshot,
                    }
                if version != last_version or payload["pose"] or payload["perception_status"]:
                    await websocket.send_json(payload)
                    last_version = version
                await asyncio.sleep(0.2)

        push_task = asyncio.create_task(push_state())
        try:
            while True:
                data = await websocket.receive_json()
                msg_type = data.get("type")
                if msg_type == "teleop":
                    linear, angular = teleop_from_keys(data.get("keys", {}))
                    bridge.publish_drive(linear, angular)
                elif msg_type == "lop":
                    bridge.request_lop()
                    await websocket.send_json({"type": "lop", "ok": True})
                elif msg_type == "mode":
                    value = bridge.set_mode(str(data.get("value", "autonomous")))
                    await websocket.send_json({"type": "mode", "value": value})
                elif msg_type == "object_recognition":
                    value = bridge.set_object_recognition(bool(data.get("value", False)))
                    await websocket.send_json({"type": "object_recognition", "value": value})
                elif msg_type == "snapshot":
                    payload = save_snapshot_frames()
                    await websocket.send_json({"type": "snapshot", **payload})
        except WebSocketDisconnect:
            bridge.stop()
        finally:
            push_task.cancel()
            bridge.stop()

    return app


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    state = SharedState()
    bridge = RosStateBridge(state)
    app = make_app(bridge, state)

    executor_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    executor_thread.start()

    try:
        host = os.environ.get("FALLITIS_WEB_HOST", "0.0.0.0")
        port = int(os.environ.get("FALLITIS_WEB_PORT", "8080"))
        uvicorn.run(app, host=host, port=port, log_level="info")
    finally:
        bridge.stop()
        bridge.destroy_node()
        rclpy.shutdown()
