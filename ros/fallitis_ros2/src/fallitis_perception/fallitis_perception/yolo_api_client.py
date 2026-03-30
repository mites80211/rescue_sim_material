from __future__ import annotations

import base64
import json
import urllib.error
import urllib.request

import cv2


class RemoteYoloApiClient:
    def __init__(
        self,
        url: str,
        *,
        conf: float,
        device: str,
        imgsz: int,
        timeout_s: float,
        jpeg_quality: int,
    ) -> None:
        self.url = url
        self.conf = conf
        self.device = device
        self.imgsz = imgsz
        self.timeout_s = timeout_s
        self.jpeg_quality = jpeg_quality

    def health_url(self) -> str:
        if self.url.endswith("/predict"):
            return f"{self.url[:-len('/predict')]}/health"
        return self.url

    def predict(self, frame_bgra: np.ndarray) -> tuple[list[dict[str, object]], dict[str, object]]:
        bgr = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
        ok, encoded = cv2.imencode(
            ".jpg",
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
        )
        if not ok:
            raise RuntimeError("Failed to encode frame for YOLO API.")

        payload = {
            "image_b64": base64.b64encode(encoded.tobytes()).decode("ascii"),
            "conf": self.conf,
            "imgsz": self.imgsz,
        }
        if self.device:
            payload["device"] = self.device
        response = self._request_json(self.url, payload)
        detections = response.get("detections", [])
        if not isinstance(detections, list):
            raise RuntimeError("YOLO API returned an invalid detections payload.")
        return detections, response

    def health(self) -> dict[str, object]:
        request = urllib.request.Request(self.health_url(), method="GET")
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            raw = response.read()
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise RuntimeError("YOLO API health endpoint returned invalid JSON.")
        return payload

    def _request_json(self, url: str, payload: dict[str, object]) -> dict[str, object]:
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            url,
            data=data,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read()
        except urllib.error.HTTPError as exc:
            details = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"YOLO API HTTP {exc.code}: {details}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"YOLO API connection failed: {exc.reason}") from exc

        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise RuntimeError("YOLO API returned invalid JSON.")
        return payload
