from __future__ import annotations

import json
import math
import os
import threading
from dataclasses import dataclass, field

import cv2
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Empty, String

from .config import WebConfig


def ros_image_to_numpy(msg: Image) -> np.ndarray:
    return np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width, 4)).copy()


def encode_jpeg(frame_bgra: np.ndarray, quality: int) -> bytes:
    bgr = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
    ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("Failed to encode JPEG frame.")
    return buffer.tobytes()


@dataclass(slots=True)
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    left_jpeg: bytes | None = None
    right_jpeg: bytes | None = None
    left_frame: np.ndarray | None = None
    right_frame: np.ndarray | None = None
    map_png: bytes | None = None
    map_frame: np.ndarray | None = None
    map_version: int = 0
    map_meta: dict[str, float | int] = field(default_factory=dict)
    pose: dict[str, float] = field(default_factory=dict)
    imu: dict[str, float] = field(default_factory=dict)
    lidar_scan: dict[str, object] = field(default_factory=dict)
    perception_status: dict[str, object] = field(default_factory=dict)
    mapping_debug: dict[str, object] = field(default_factory=dict)
    actuation_status: dict[str, object] = field(default_factory=dict)
    last_snapshot: str = ""


class RosStateBridge(Node):
    def __init__(self, state: SharedState) -> None:
        super().__init__("fallitis_web_bridge")
        self.state = state
        self.config = WebConfig.from_node(self)
        self.jpeg_quality = max(80, min(100, int(os.environ.get("FALLITIS_WEB_JPEG_QUALITY", "96"))))
        self.teleop_pub = self.create_publisher(Twist, "/teleop/cmd_vel_input", 10)
        self.lop_pub = self.create_publisher(Empty, "/teleop/lack_of_progress", 10)
        self.mode_pub = self.create_publisher(String, "/teleop/mode", 10)
        self.object_recognition_pub = self.create_publisher(Bool, "/teleop/object_recognition_enabled", 10)

        self.create_subscription(Image, "/robot/camera/left/image_raw", self.on_left_image, 5)
        self.create_subscription(Image, "/robot/camera/right/image_raw", self.on_right_image, 5)
        self.create_subscription(LaserScan, "/robot/lidar/scan", self.on_lidar_scan, 5)
        self.create_subscription(OccupancyGrid, "/mapping/map", self.on_map, 5)
        self.create_subscription(Odometry, "/robot/odometry", self.on_odometry, 10)
        self.create_subscription(Vector3Stamped, "/robot/imu/rpy", self.on_imu, 10)
        self.create_subscription(String, "/perception/status", self.on_perception_status, 10)
        self.create_subscription(String, "/mapping/debug", self.on_mapping_debug, 10)
        self.create_subscription(String, "/actuation/status", self.on_actuation_status, 10)

    def publish_drive(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.teleop_pub.publish(msg)

    def stop(self) -> None:
        self.publish_drive(0.0, 0.0)

    def request_lop(self) -> None:
        self.lop_pub.publish(Empty())

    def set_mode(self, mode: str) -> str:
        selected = mode.strip().lower()
        if selected not in {"manual", "autonomous"}:
            raise ValueError(f"Unsupported control mode: {mode}")
        self.mode_pub.publish(String(data=selected))
        with self.state.lock:
            self.state.actuation_status = {**self.state.actuation_status, "mode": selected}
        if selected != "manual":
            self.stop()
        return selected

    def set_object_recognition(self, enabled: bool) -> bool:
        selected = bool(enabled)
        self.object_recognition_pub.publish(Bool(data=selected))
        with self.state.lock:
            self.state.perception_status = {
                **self.state.perception_status,
                "object_recognition_enabled": selected,
            }
        return selected

    def on_left_image(self, msg: Image) -> None:
        frame = ros_image_to_numpy(msg)
        jpeg = encode_jpeg(frame, self.jpeg_quality)
        with self.state.lock:
            self.state.left_frame = frame
            self.state.left_jpeg = jpeg

    def on_right_image(self, msg: Image) -> None:
        frame = ros_image_to_numpy(msg)
        jpeg = encode_jpeg(frame, self.jpeg_quality)
        with self.state.lock:
            self.state.right_frame = frame
            self.state.right_jpeg = jpeg

    def on_lidar_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            payload = {"points": [], "range_max": float(msg.range_max), "sample_count": 0}
        else:
            sample_count = min(360, int(ranges.size))
            indices = np.linspace(0, ranges.size - 1, sample_count, dtype=int)
            points: list[list[float]] = []
            for idx in indices:
                distance = float(ranges[idx])
                if not math.isfinite(distance):
                    continue
                angle = float(msg.angle_min + idx * msg.angle_increment)
                points.append([distance * math.cos(angle), distance * math.sin(angle)])
            payload = {
                "points": points,
                "range_max": float(msg.range_max),
                "sample_count": sample_count,
            }
        with self.state.lock:
            self.state.lidar_scan = payload

    def on_map(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        rendered = np.zeros((height, width, 3), dtype=np.uint8)
        unknown = data < 0
        free = (data >= 0) & (data <= 35)
        occupied = data > 35
        rendered[unknown] = (38, 44, 54)
        rendered[free] = (229, 231, 235)
        rendered[occupied] = (200, 85, 72)
        rendered = rendered[::-1]
        ok, buffer = cv2.imencode(".png", rendered)
        if not ok:
            return
        with self.state.lock:
            self.state.map_frame = rendered
            self.state.map_png = buffer.tobytes()
            self.state.map_version += 1
            self.state.map_meta = {
                "width": width,
                "height": height,
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
            }

    def on_odometry(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        with self.state.lock:
            self.state.pose = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw_qz": float(pose.orientation.z),
                "yaw_qw": float(pose.orientation.w),
                "linear_x": float(msg.twist.twist.linear.x),
                "angular_z": float(msg.twist.twist.angular.z),
            }

    def on_imu(self, msg: Vector3Stamped) -> None:
        with self.state.lock:
            self.state.imu = {
                "roll": float(msg.vector.x),
                "pitch": float(msg.vector.y),
                "yaw": float(msg.vector.z),
            }

    def on_perception_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}
        with self.state.lock:
            self.state.perception_status = payload

    def on_mapping_debug(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}
        with self.state.lock:
            self.state.mapping_debug = payload

    def on_actuation_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}
        with self.state.lock:
            self.state.actuation_status = payload
