#!/usr/bin/env python3
"""Minimal YOLO HTTP API server.

Dependencies:
  pip install ultralytics

Example:
  python3 yolo_api.py --model yolov8n.pt --host 0.0.0.0 --port 8000

Endpoints:
  GET  /health
  POST /predict

POST /predict JSON body:
  {
    "image_path": "/path/to/image.jpg",
    "conf": 0.25,
    "imgsz": 640
  }

Or:
  {
    "image_b64": "<base64 image>",
    "conf": 0.25
  }
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import tempfile
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


class YoloServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        *,
        model_path: str,
        device: str | None,
        default_conf: float,
        default_imgsz: int,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.model_path = model_path
        self.device = device
        self.default_conf = default_conf
        self.default_imgsz = default_imgsz
        self.started_at = time.time()
        self._model_lock = threading.Lock()
        self._predict_lock = threading.Lock()
        self._model = None

    def load_model(self) -> Any:
        if self._model is not None:
            return self._model

        with self._model_lock:
            if self._model is not None:
                return self._model

            try:
                from ultralytics import YOLO
            except ImportError as exc:
                raise RuntimeError(
                    "Missing dependency 'ultralytics'. Install it with: pip install ultralytics"
                ) from exc

            self._model = YOLO(self.model_path)
        return self._model


class RequestHandler(BaseHTTPRequestHandler):
    server: YoloServer

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self._send_cors_headers()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self) -> None:
        if self.path == "/" or self.path == "/health":
            self._send_json(
                HTTPStatus.OK,
                {
                    "status": "ok",
                    "model": self.server.model_path,
                    "device": self.server.device or "auto",
                    "loaded": self.server._model is not None,
                    "uptime_s": round(time.time() - self.server.started_at, 3),
                },
            )
            return

        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:
        if self.path != "/predict":
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return

        try:
            payload = self._read_json()
            response = self._predict(payload)
            self._send_json(HTTPStatus.OK, response)
        except ValueError as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
        except Exception as exc:  # noqa: BLE001
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})

    def log_message(self, fmt: str, *args: Any) -> None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        message = fmt % args
        print(f"[{timestamp}] {self.address_string()} {message}")

    def _read_json(self) -> dict[str, Any]:
        content_length = self.headers.get("Content-Length")
        if not content_length:
            raise ValueError("Missing Content-Length")

        try:
            size = int(content_length)
        except ValueError as exc:
            raise ValueError("Invalid Content-Length") from exc

        raw_body = self.rfile.read(size)
        try:
            payload = json.loads(raw_body)
        except json.JSONDecodeError as exc:
            raise ValueError("Request body must be valid JSON") from exc

        if not isinstance(payload, dict):
            raise ValueError("Request body must be a JSON object")
        return payload

    def _predict(self, payload: dict[str, Any]) -> dict[str, Any]:
        image_path = payload.get("image_path")
        image_b64 = payload.get("image_b64")
        if bool(image_path) == bool(image_b64):
            raise ValueError("Provide exactly one of 'image_path' or 'image_b64'")

        conf = float(payload.get("conf", self.server.default_conf))
        imgsz = int(payload.get("imgsz", self.server.default_imgsz))
        device = payload.get("device", self.server.device)

        temp_path: str | None = None
        source = image_path

        if image_b64:
            temp_path = self._decode_base64_image(str(image_b64))
            source = temp_path
        else:
            source = str(image_path)
            if not os.path.exists(source):
                raise ValueError(f"image_path does not exist: {source}")

        started = time.perf_counter()
        try:
            model = self.server.load_model()
            with self.server._predict_lock:
                results = model.predict(
                    source=source,
                    conf=conf,
                    imgsz=imgsz,
                    device=device,
                    verbose=False,
                )
        finally:
            if temp_path:
                try:
                    os.unlink(temp_path)
                except FileNotFoundError:
                    pass

        elapsed_ms = round((time.perf_counter() - started) * 1000, 3)
        if not results:
            raise RuntimeError("Model returned no results")

        result = results[0]
        detections = []
        names = result.names
        boxes = result.boxes

        if boxes is not None:
            total = len(boxes)
            for index in range(total):
                cls_id = int(boxes.cls[index].item())
                if isinstance(names, dict):
                    class_name = names.get(cls_id, str(cls_id))
                else:
                    class_name = names[cls_id]

                detections.append(
                    {
                        "class_id": cls_id,
                        "class_name": class_name,
                        "confidence": round(float(boxes.conf[index].item()), 6),
                        "xyxy": [round(float(v), 3) for v in boxes.xyxy[index].tolist()],
                    }
                )

        return {
            "model": self.server.model_path,
            "device": device or "auto",
            "image": str(image_path) if image_path else "<base64>",
            "shape": {
                "height": int(result.orig_shape[0]),
                "width": int(result.orig_shape[1]),
            },
            "count": len(detections),
            "elapsed_ms": elapsed_ms,
            "detections": detections,
        }

    def _decode_base64_image(self, image_b64: str) -> str:
        suffix = ".jpg"
        if "," in image_b64 and image_b64.startswith("data:"):
            header, image_b64 = image_b64.split(",", 1)
            mime = header[5:].split(";", 1)[0].strip().lower()
            suffix = {
                "image/jpeg": ".jpg",
                "image/jpg": ".jpg",
                "image/png": ".png",
                "image/webp": ".webp",
            }.get(mime, suffix)

        try:
            data = base64.b64decode(image_b64, validate=True)
        except Exception as exc:  # noqa: BLE001
            raise ValueError("Invalid image_b64 payload") from exc

        if data.startswith(b"\x89PNG\r\n\x1a\n"):
            suffix = ".png"
        elif data.startswith(b"\xff\xd8\xff"):
            suffix = ".jpg"
        elif data[:4] == b"RIFF" and data[8:12] == b"WEBP":
            suffix = ".webp"

        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=suffix)
        try:
            temp_file.write(data)
            temp_file.flush()
        finally:
            temp_file.close()
        return temp_file.name

    def _send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self._send_cors_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_cors_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")


def parse_args() -> argparse.Namespace:
    env_model = os.environ.get("YOLO_MODEL")
    if env_model:
        default_model = env_model
    elif os.path.exists("yolo.pt"):
        default_model = str(Path("yolo.pt").resolve())
    else:
        default_model = "yolov8n.pt"

    parser = argparse.ArgumentParser(description="Serve a YOLO model over HTTP")
    parser.add_argument(
        "--model",
        default=default_model,
        help="Path or model name to load (default: %(default)s)",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("YOLO_HOST", "0.0.0.0"),
        help="Bind host (default: %(default)s)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("YOLO_PORT", "8000")),
        help="Bind port (default: %(default)s)",
    )
    parser.add_argument(
        "--device",
        default=os.environ.get("YOLO_DEVICE"),
        help="Torch device, for example 'cpu', 'cuda', or '0'",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=float(os.environ.get("YOLO_CONF", "0.25")),
        help="Default confidence threshold (default: %(default)s)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=int(os.environ.get("YOLO_IMGSZ", "640")),
        help="Default inference size (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    model_ref = str(Path(args.model)) if os.path.exists(args.model) else args.model
    server = YoloServer(
        (args.host, args.port),
        RequestHandler,
        model_path=model_ref,
        device=args.device,
        default_conf=args.conf,
        default_imgsz=args.imgsz,
    )

    print(
        f"YOLO API listening on http://{args.host}:{args.port} "
        f"using model={model_ref} device={args.device or 'auto'}"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
