from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .config import PerceptionConfig
from .image_tools import annotate_status, draw_detection_boxes, numpy_to_ros_image, ros_image_to_numpy
from .yolo_api_client import RemoteYoloApiClient


def finite_or_none(value: float) -> float | None:
    return float(value) if np.isfinite(value) else None


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("fallitis_perception")
        self.config = PerceptionConfig.from_node(self)

        self.status_pub = self.create_publisher(String, "/perception/status", 5)
        self.left_annotated_pub = self.create_publisher(Image, "/perception/camera/left/annotated", 5)
        self.right_annotated_pub = self.create_publisher(Image, "/perception/camera/right/annotated", 5)

        self.create_subscription(Image, "/robot/camera/left/image_raw", self.on_left_image, 5)
        self.create_subscription(Image, "/robot/camera/right/image_raw", self.on_right_image, 5)
        self.create_subscription(Image, "/robot/lidar/range_image", self.on_lidar_range_image, 10)
        self.create_subscription(LaserScan, "/robot/lidar/scan", self.on_scan, 10)

        self.last_left_image: Image | None = None
        self.last_right_image: Image | None = None
        self.last_detection_count = 0
        self.last_detection_summary = "No frames yet"
        self.last_detector_contact_ts = 0.0
        self.last_lidar_valid_count = 0
        self.last_lidar_min_range = float("nan")
        self.last_lidar_max_range = float("nan")
        self.last_scan_front_range = float("nan")
        self.last_left_shape = (0, 0)
        self.last_right_shape = (0, 0)

        self.yolo_client: RemoteYoloApiClient | None = None
        self.yolo_status = "YOLO API disabled"
        self._init_yolo_api()

        self.annotation_timer = self.create_timer(self.config.annotation_period_s, self.publish_annotations)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.health_timer = self.create_timer(3.0, self.refresh_yolo_status)
        self.step_mode_enabled = False
        self.annotation_every_steps = 1
        self.status_every_steps = 1
        self.health_every_steps = 1

        self.get_logger().info("Perception node initialised.")

    def _init_yolo_api(self) -> None:
        if not self.config.yolo_api_enabled or not self.config.yolo_api_url:
            self.yolo_status = "YOLO API disabled"
            return
        self.yolo_client = RemoteYoloApiClient(
            self.config.yolo_api_url,
            conf=self.config.model_conf,
            device=self.config.model_device,
            imgsz=self.config.model_imgsz,
            timeout_s=self.config.yolo_api_timeout_s,
            jpeg_quality=self.config.yolo_api_jpeg_quality,
        )
        self.refresh_yolo_status()

    def on_left_image(self, msg: Image) -> None:
        self.last_left_image = msg
        self.last_left_shape = (int(msg.width), int(msg.height))

    def on_right_image(self, msg: Image) -> None:
        self.last_right_image = msg
        self.last_right_shape = (int(msg.width), int(msg.height))

    def on_lidar_range_image(self, msg: Image) -> None:
        if msg.encoding != "32FC1":
            return
        ranges = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        finite = np.isfinite(ranges)
        self.last_lidar_valid_count = int(np.count_nonzero(finite))
        if self.last_lidar_valid_count == 0:
            self.last_lidar_min_range = float("nan")
            self.last_lidar_max_range = float("nan")
            return
        values = ranges[finite]
        self.last_lidar_min_range = float(np.min(values))
        self.last_lidar_max_range = float(np.max(values))

    def on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        finite = np.isfinite(ranges)
        if not np.any(finite):
            self.last_scan_front_range = float("nan")
            return
        front_idx = len(ranges) // 2
        window = ranges[max(0, front_idx - 4) : min(len(ranges), front_idx + 5)]
        finite_window = window[np.isfinite(window)]
        self.last_scan_front_range = float(np.min(finite_window)) if finite_window.size else float("nan")

    def refresh_yolo_status(self) -> None:
        if self.yolo_client is None:
            self.yolo_status = "YOLO API disabled"
            return
        try:
            health = self.yolo_client.health()
            model = health.get("model", "<unknown>")
            loaded = health.get("loaded", False)
            device = health.get("device", "auto")
            self.yolo_status = f"YOLO API ok: model={model} device={device} loaded={loaded}"
            self.last_detector_contact_ts = time.time()
        except Exception as exc:  # noqa: BLE001
            self.yolo_status = f"YOLO API unavailable: {exc}"

    def detect(self, frame_bgra):
        if self.yolo_client is None:
            return frame_bgra, []
        detections, response = self.yolo_client.predict(frame_bgra)
        self.last_detector_contact_ts = time.time()
        backend = response.get("model", "remote")
        self.yolo_status = f"YOLO API ok: model={backend} count={len(detections)}"
        if not detections:
            return frame_bgra, []
        annotated = draw_detection_boxes(frame_bgra, detections)
        return annotated, detections

    def publish_annotations(self) -> None:
        images = [
            ("left", self.last_left_image, self.left_annotated_pub),
            ("right", self.last_right_image, self.right_annotated_pub),
        ]
        detection_count = 0
        summary_parts: list[str] = []
        for label, ros_image, publisher in images:
            if ros_image is None:
                continue
            frame = ros_image_to_numpy(ros_image)
            try:
                annotated, detections = self.detect(frame)
            except Exception as exc:  # noqa: BLE001
                annotated = frame
                detections = []
                self.yolo_status = f"YOLO inference error: {exc}"
            detection_count += len(detections)
            summary_parts.append(f"{label}:{len(detections)}")
            lines = [
                self.yolo_status,
                f"front {self.last_scan_front_range:.3f}m valid {self.last_lidar_valid_count}",
            ]
            annotated = annotate_status(annotated, f"{label.upper()} camera", lines)
            publisher.publish(numpy_to_ros_image(annotated, ros_image.header.frame_id, ros_image.header.stamp))
        self.last_detection_count = detection_count
        if summary_parts:
            self.last_detection_summary = ", ".join(summary_parts)

    def publish_status(self) -> None:
        payload = {
            "yolo_enabled": self.yolo_client is not None,
            "yolo_backend": "remote_api" if self.yolo_client is not None else "disabled",
            "yolo_api_url": self.config.yolo_api_url if self.yolo_client is not None else "",
            "yolo_status": self.yolo_status,
            "last_detection_count": self.last_detection_count,
            "last_detection_summary": self.last_detection_summary,
            "last_detector_contact_ts": self.last_detector_contact_ts,
            "left_camera_shape": self.last_left_shape,
            "right_camera_shape": self.last_right_shape,
            "lidar_valid_count": self.last_lidar_valid_count,
            "lidar_min_range": finite_or_none(self.last_lidar_min_range),
            "lidar_max_range": finite_or_none(self.last_lidar_max_range),
            "lidar_front_range": finite_or_none(self.last_scan_front_range),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def enable_step_mode(self, time_step_s: float) -> None:
        self.step_mode_enabled = True
        self.annotation_every_steps = max(1, round(self.config.annotation_period_s / max(time_step_s, 1e-6)))
        self.status_every_steps = max(1, round(0.5 / max(time_step_s, 1e-6)))
        self.health_every_steps = max(1, round(3.0 / max(time_step_s, 1e-6)))
        self.annotation_timer.cancel()
        self.status_timer.cancel()
        self.health_timer.cancel()

    def step_update(self, step_index: int) -> None:
        if not self.step_mode_enabled:
            return
        if step_index % self.health_every_steps == 0:
            self.refresh_yolo_status()
        if step_index % self.annotation_every_steps == 0:
            self.publish_annotations()
        if step_index % self.status_every_steps == 0:
            self.publish_status()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
