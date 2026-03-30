from __future__ import annotations

import json
from collections import deque
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Empty, Int32, String

from .config import MappingConfig
from .gps_filtering import LidarMotionEstimate, LidarMotionTracker, WheelOdometryTracker, stamp_to_seconds
from .occupancy import LidarDataProcessor, OccupancyGridModel, yaw_from_quaternion
from .pose_estimator import OdomLidarPoseEstimator, PoseEstimate


def sample_points(points: deque[tuple[float, float]], limit: int) -> list[list[float]]:
    if not points:
        return []
    if len(points) <= limit:
        return [[float(x), float(y)] for x, y in points]
    step = max(1, len(points) // limit)
    sampled = list(points)[::step]
    return [[float(x), float(y)] for x, y in sampled[-limit:]]


def finite_or_none(value: float) -> float | None:
    return float(value) if np.isfinite(value) else None


class MappingNode(Node):
    def __init__(self) -> None:
        super().__init__("fallitis_mapping")
        self.config = MappingConfig.from_node(self)

        self.map_shape = (
            self.config.map_width_cells,
            self.config.map_height_cells,
        )
        self.map_spacing = self.config.map_spacing
        self.lidar_max_range_m = self.config.lidar_max_range_m
        self.trail_length = self.config.trail_length

        self.map_pub = self.create_publisher(OccupancyGrid, "/mapping/map", 5)
        self.filtered_odom_pub = self.create_publisher(Odometry, "/robot/odometry", 10)
        self.debug_pub = self.create_publisher(String, "/mapping/debug", 5)

        self.create_subscription(Odometry, "/robot/odometry_raw", self.on_raw_odometry, 10)
        self.create_subscription(JointState, "/robot/wheel_states", self.on_wheel_state, 10)
        self.create_subscription(LaserScan, "/robot/lidar/scan", self.on_scan, 10)
        self.create_subscription(Image, "/robot/lidar/range_image", self.on_lidar_range_image, 10)
        self.create_subscription(Empty, "/robot/lack_of_progress", self.on_lack_of_progress, 10)
        self.create_subscription(Int32, "/mapping/gps_filter_window", self.on_filter_window, 10)

        self.map_model: OccupancyGridModel | None = None
        self.lidar_processor: LidarDataProcessor | None = None
        self.raw_pose: dict[str, float] | None = None
        self.filtered_pose: dict[str, float] | None = None
        self.pose_history: deque[dict[str, float]] = deque(maxlen=self.config.pose_history_size)
        self.last_range_image: np.ndarray | None = None
        self.last_scan_meta: LidarMotionEstimate = LidarMotionEstimate()
        self.wheel_tracker = WheelOdometryTracker(
            wheel_radius=self.config.wheel_radius,
            axle_length=self.config.axle_length,
        )
        self.pose_estimator = OdomLidarPoseEstimator(
            window_size=self.config.gps_filter_window,
            expected_dt_s=self.config.expected_dt_s,
            spawn_anchor_sample_count=self.config.spawn_anchor_sample_count,
            spawn_snap_step_m=self.config.spawn_snap_step_m,
            linear_stationary_threshold_m_s=self.config.linear_stationary_threshold_m_s,
            linear_motion_threshold_m_s=self.config.linear_motion_threshold_m_s,
            angular_stationary_threshold_rad_s=self.config.angular_stationary_threshold_rad_s,
            angular_motion_threshold_rad_s=self.config.angular_motion_threshold_rad_s,
            imu_yaw_rate_stationary_threshold_rad_s=self.config.imu_yaw_rate_stationary_threshold_rad_s,
            lidar_static_confidence_threshold=self.config.lidar_static_confidence_threshold,
            predictive_index_threshold=self.config.predictive_index_threshold,
        )
        self.lidar_motion_tracker = LidarMotionTracker(
            close_range_m=self.config.lidar_close_range_m,
            stationary_range_eps_m=self.config.lidar_stationary_range_eps_m,
            downsample=self.config.lidar_downsample,
        )
        self.raw_trail: deque[tuple[float, float]] = deque(maxlen=self.trail_length)
        self.filtered_trail: deque[tuple[float, float]] = deque(maxlen=self.trail_length)
        self.last_filter_debug: dict[str, object] = {}
        self.filter_reset_count = 0
        self.last_filter_reset_s = 0.0

        self.debug_timer = self.create_timer(self.config.debug_publish_period_s, self.publish_debug)
        self.step_mode_enabled = False
        self.debug_every_steps = 1

    def on_filter_window(self, msg: Int32) -> None:
        value = max(self.config.gps_filter_window_min, min(self.config.gps_filter_window_max, int(msg.data)))
        self.pose_estimator.set_window_size(value)
        self.get_logger().info(f"Updated motion window to {self.pose_estimator.window_size}")

    def on_wheel_state(self, msg: JointState) -> None:
        self.wheel_tracker.update(msg)

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan_meta = self.lidar_motion_tracker.update(
            msg,
            self.wheel_tracker.latest.linear_velocity,
        )

    def reset_runtime_state(self, reason: str, *, clear_map: bool) -> None:
        self.pose_estimator.reset(self.config.gps_filter_window)
        self.lidar_motion_tracker.reset()
        self.raw_trail.clear()
        self.filtered_trail.clear()
        self.pose_history.clear()
        self.raw_pose = None
        self.filtered_pose = None
        if clear_map:
            self.map_model = None
            self.lidar_processor = None
            self.last_range_image = None
        self.last_filter_debug = {
            "reset_reason": reason,
            "gps_anchor_pending": True,
            "window_size": self.config.gps_filter_window,
        }
        self.filter_reset_count += 1
        self.last_filter_reset_s = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"{reason}: support stack state reset.")

    def on_lack_of_progress(self, _: Empty) -> None:
        self.reset_runtime_state("lop", clear_map=True)

    def should_reset_for_raw_jump(self, raw_x: float, raw_y: float) -> bool:
        if self.raw_pose is None:
            return False
        dx = raw_x - self.raw_pose["x"]
        dy = raw_y - self.raw_pose["y"]
        jump_distance = math.hypot(dx, dy)
        return jump_distance >= self.config.robot_reset_jump_threshold_m

    def on_raw_odometry(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        raw_x = float(pose.position.x)
        raw_y = float(pose.position.y)
        stamp_s = stamp_to_seconds(msg.header.stamp)
        if self.should_reset_for_raw_jump(raw_x, raw_y):
            self.reset_runtime_state("robot_reset_detected", clear_map=True)
        self.raw_pose = {"x": raw_x, "y": raw_y, "yaw": yaw}
        self.raw_trail.append((raw_x, raw_y))

        estimate = self.pose_estimator.update(
            raw_x=raw_x,
            raw_y=raw_y,
            yaw=yaw,
            stamp_s=stamp_s,
            wheel_motion=self.wheel_tracker.latest,
            lidar_motion=self.last_scan_meta,
        )
        if estimate is None:
            self.last_filter_debug = self.build_pending_filter_debug(raw_x, raw_y)
            return
        self.last_filter_debug = self.build_filter_debug(estimate)
        self.filtered_pose = {
            "x": float(estimate.x),
            "y": float(estimate.y),
            "yaw": float(estimate.yaw),
        }
        self.pose_history.append(
            {
                "stamp_s": float(stamp_s),
                "x": float(estimate.x),
                "y": float(estimate.y),
                "yaw": float(estimate.yaw),
            }
        )
        self.filtered_trail.append((self.filtered_pose["x"], self.filtered_pose["y"]))
        self.filtered_odom_pub.publish(self.build_filtered_odometry(msg, estimate))

    def on_lidar_range_image(self, msg: Image) -> None:
        if msg.encoding != "32FC1":
            return
        self.last_range_image = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width)).copy()
        pose_for_scan = self.lookup_pose_for_stamp(stamp_to_seconds(msg.header.stamp))
        if pose_for_scan is None:
            return
        if self.map_model is None:
            self.map_model = OccupancyGridModel(
                self.map_shape,
                self.map_spacing,
                (pose_for_scan["x"], pose_for_scan["y"]),
            )
            self.lidar_processor = LidarDataProcessor(
                max_range_m=self.lidar_max_range_m,
                hor_res=msg.width,
                hor_fov=2.0 * np.pi,
                ver_res=msg.height,
                ver_fov=0.2,
                yaw_offset_rad=self.config.map_yaw_offset_rad,
                occupancy_grid=self.map_model,
            )
        self.update_map(msg.header.stamp, pose_for_scan)

    def lookup_pose_for_stamp(self, stamp_s: float) -> dict[str, float] | None:
        if not self.pose_history:
            return self.filtered_pose
        closest = min(self.pose_history, key=lambda pose: abs(pose["stamp_s"] - stamp_s))
        if abs(closest["stamp_s"] - stamp_s) > self.config.pose_sync_tolerance_s:
            return None
        return closest

    def build_filter_debug(self, estimate: PoseEstimate) -> dict[str, object]:
        return {
            "gps_mode": "spawn_and_lop_only",
            "imu_mode": "direct_yaw_from_inertial_unit",
            "dt_s": float(estimate.dt_s),
            "wheel_linear_velocity": float(estimate.wheel_linear_velocity),
            "wheel_angular_velocity": float(estimate.wheel_angular_velocity),
            "imu_yaw_rate_rad_s": float(estimate.imu_yaw_rate_rad_s),
            "predictive_index": float(estimate.predictive_index),
            "predictive_sample": float(estimate.predictive_sample),
            "block_index": float(estimate.block_index),
            "lidar_motion_index": float(estimate.lidar_motion_index),
            "translation_distance_m": float(estimate.translation_distance_m),
            "translation_enabled": bool(estimate.translation_enabled),
            "translation_frozen": bool(estimate.translation_frozen),
            "stationary": bool(estimate.stationary),
            "rotating_in_place": bool(estimate.rotating_in_place),
            "slip_likely": bool(estimate.slip_likely),
            "gps_anchor_used": bool(estimate.gps_anchor_used),
            "gps_anchor_pending": bool(estimate.gps_anchor_pending),
            "spawn_anchor": [float(estimate.spawn_x), float(estimate.spawn_y)],
            "spawn_snap_step_m": float(self.config.spawn_snap_step_m),
            "raw_gps_pose": [float(estimate.raw_x), float(estimate.raw_y)],
            "window_size": int(estimate.window_size),
            "pose_sync_tolerance_s": float(self.config.pose_sync_tolerance_s),
            "map_yaw_offset_rad": float(self.config.map_yaw_offset_rad),
        }

    def build_pending_filter_debug(self, raw_x: float, raw_y: float) -> dict[str, object]:
        buffered_x, buffered_y = self.pose_estimator.pending_anchor_pose(raw_x, raw_y)
        return {
            "gps_mode": "spawn_and_lop_only",
            "imu_mode": "direct_yaw_from_inertial_unit",
            "gps_anchor_pending": True,
            "gps_anchor_samples": self.pose_estimator.anchor_sample_count,
            "gps_anchor_sample_target": self.pose_estimator.anchor_sample_target,
            "buffered_spawn_raw_pose": [float(buffered_x), float(buffered_y)],
            "spawn_snap_step_m": float(self.config.spawn_snap_step_m),
            "window_size": int(self.pose_estimator.window_size),
            "pose_sync_tolerance_s": float(self.config.pose_sync_tolerance_s),
            "map_yaw_offset_rad": float(self.config.map_yaw_offset_rad),
        }

    def build_filtered_odometry(self, source_msg: Odometry, estimate: PoseEstimate) -> Odometry:
        odom = Odometry()
        odom.header = source_msg.header
        odom.child_frame_id = source_msg.child_frame_id or "base_link"
        odom.pose.pose = source_msg.pose.pose
        odom.pose.pose.position.x = float(estimate.x)
        odom.pose.pose.position.y = float(estimate.y)
        odom.twist.twist = source_msg.twist.twist
        odom.twist.twist.linear.x = float(estimate.wheel_linear_velocity if estimate.translation_enabled else 0.0)
        odom.twist.twist.angular.z = float(estimate.imu_yaw_rate_rad_s)
        return odom

    def update_map(self, stamp, pose: dict[str, float]) -> None:
        if self.last_range_image is None or self.map_model is None or self.lidar_processor is None:
            return
        scan = self.lidar_processor.lidar_scan(self.last_range_image, pose["yaw"])
        center = self.map_model.world_to_grid(pose["x"], pose["y"])
        self.map_model.update_region(scan, center)
        self.map_pub.publish(self.build_map_message(stamp))

    def build_map_message(self, stamp) -> OccupancyGrid:
        if self.map_model is None:
            raise RuntimeError("Map model is not initialised.")
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = "map"
        grid.info.resolution = 1.0 / float(self.map_model.spacing)
        grid.info.width = self.map_model.shape[0]
        grid.info.height = self.map_model.shape[1]
        origin_x, origin_y = self.map_model.grid_to_world(0, self.map_model.shape[1] - 1)
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0
        data = np.flip(self.map_model.occupancy_data().T, axis=0)
        grid.data = data.reshape(-1).astype(int).tolist()
        return grid

    def publish_debug(self) -> None:
        payload = {
            "raw_pose": self.raw_pose,
            "filtered_pose": self.filtered_pose,
            "trail_raw": sample_points(self.raw_trail, 320),
            "trail_filtered": sample_points(self.filtered_trail, 320),
            "gps_filter_window": self.pose_estimator.window_size,
            "gps_filter_window_default": self.config.gps_filter_window,
            "gps_filter_window_min": self.config.gps_filter_window_min,
            "gps_filter_window_max": self.config.gps_filter_window_max,
            "filter_reset_count": self.filter_reset_count,
            "last_filter_reset_s": self.last_filter_reset_s,
            "wheel_linear_velocity": self.wheel_tracker.latest.linear_velocity,
            "wheel_angular_velocity": self.wheel_tracker.latest.angular_velocity,
            "wheel_left_rad_s": self.wheel_tracker.latest.left_rad_s,
            "wheel_right_rad_s": self.wheel_tracker.latest.right_rad_s,
            "lidar_stationary_confidence": finite_or_none(self.last_scan_meta.stationary_confidence),
            "lidar_median_delta_m": finite_or_none(self.last_scan_meta.median_delta_m),
            "lidar_directional_motion_delta_m": finite_or_none(self.last_scan_meta.directional_motion_delta_m),
            "lidar_close_point_count": self.last_scan_meta.close_point_count,
            "lidar_directional_ranges": {
                label: finite_or_none(value) if value is not None else None
                for label, value in self.last_scan_meta.directional_ranges.items()
            },
            "lidar_directional_deltas": {
                label: finite_or_none(value) if value is not None else None
                for label, value in self.last_scan_meta.directional_deltas.items()
            },
            "lidar_range_max_m": finite_or_none(self.last_scan_meta.range_max_m),
            "lidar_points": [[float(x), float(y)] for x, y in self.last_scan_meta.points_xy[:220]],
            "filter_debug": self.last_filter_debug,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.debug_pub.publish(msg)

    def enable_step_mode(self, time_step_s: float) -> None:
        self.step_mode_enabled = True
        self.debug_every_steps = max(1, round(self.config.debug_publish_period_s / max(time_step_s, 1e-6)))
        self.debug_timer.cancel()

    def step_update(self, step_index: int) -> None:
        if self.step_mode_enabled and step_index % self.debug_every_steps == 0:
            self.publish_debug()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
