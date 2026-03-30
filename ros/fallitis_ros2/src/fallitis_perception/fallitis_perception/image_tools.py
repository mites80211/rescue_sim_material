from __future__ import annotations

import time

import cv2
import numpy as np
from sensor_msgs.msg import Image


def ros_image_to_numpy(msg: Image) -> np.ndarray:
    if msg.encoding != "bgra8":
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    frame = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width, 4)).copy()
    return frame


def numpy_to_ros_image(frame: np.ndarray, frame_id: str, stamp) -> Image:
    if frame.ndim != 3 or frame.shape[2] != 4:
        raise ValueError("Expected a BGRA image.")
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = "bgra8"
    msg.is_bigendian = False
    msg.step = int(frame.shape[1] * frame.shape[2])
    msg.data = frame.tobytes()
    return msg


def ensure_bgra(frame: np.ndarray) -> np.ndarray:
    if frame.shape[2] == 4:
        return frame
    return cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)


def annotate_status(frame: np.ndarray, title: str, lines: list[str]) -> np.ndarray:
    bgra = ensure_bgra(frame.copy())
    bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 70), (25, 28, 34), -1)
    cv2.putText(bgr, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (235, 241, 245), 2, cv2.LINE_AA)
    for idx, line in enumerate(lines[:2]):
        cv2.putText(
            bgr,
            line,
            (12, 48 + idx * 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (147, 197, 253),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        bgr,
        time.strftime("%H:%M:%S"),
        (bgr.shape[1] - 92, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 214, 102),
        2,
        cv2.LINE_AA,
    )
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA)


def draw_detection_boxes(frame: np.ndarray, detections: list[dict[str, object]]) -> np.ndarray:
    bgra = ensure_bgra(frame.copy())
    bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    for detection in detections:
        x1, y1, x2, y2 = [int(v) for v in detection["xyxy"]]
        label = f"{detection['class_name']} {float(detection['confidence']):.2f}"
        cv2.rectangle(bgr, (x1, y1), (x2, y2), (55, 184, 123), 2)
        cv2.rectangle(bgr, (x1, max(0, y1 - 24)), (min(bgr.shape[1], x1 + 190), y1), (55, 184, 123), -1)
        cv2.putText(bgr, label, (x1 + 4, max(16, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (10, 18, 22), 1)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA)
