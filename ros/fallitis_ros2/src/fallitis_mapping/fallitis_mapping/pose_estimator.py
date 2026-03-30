from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from .gps_filtering import LidarMotionEstimate, WheelMotionEstimate, clamp
from .spawn_anchor import SpawnAnchorAccumulator


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_delta(current: float, previous: float) -> float:
    return wrap_angle(current - previous)


def mean_or_zero(values: deque[float]) -> float:
    return sum(values) / float(len(values)) if values else 0.0


def forward_vector_from_yaw(yaw: float) -> tuple[float, float]:
    return (math.sin(yaw), math.cos(yaw))


@dataclass(slots=True)
class PoseEstimate:
    x: float
    y: float
    yaw: float
    dt_s: float
    wheel_linear_velocity: float
    wheel_angular_velocity: float
    imu_yaw_rate_rad_s: float
    predictive_index: float
    predictive_sample: float
    block_index: float
    lidar_motion_index: float
    translation_distance_m: float
    translation_enabled: bool
    translation_frozen: bool
    stationary: bool
    rotating_in_place: bool
    slip_likely: bool
    gps_anchor_used: bool
    gps_anchor_pending: bool
    spawn_x: float
    spawn_y: float
    raw_x: float
    raw_y: float
    window_size: int


class OdomLidarPoseEstimator:
    def __init__(
        self,
        *,
        window_size: int,
        expected_dt_s: float,
        spawn_anchor_sample_count: int,
        spawn_snap_step_m: float,
        linear_stationary_threshold_m_s: float,
        linear_motion_threshold_m_s: float,
        angular_stationary_threshold_rad_s: float,
        angular_motion_threshold_rad_s: float,
        imu_yaw_rate_stationary_threshold_rad_s: float,
        lidar_static_confidence_threshold: float,
        predictive_index_threshold: float,
    ) -> None:
        self.expected_dt_s = float(expected_dt_s)
        self.default_window_size = max(1, int(window_size))
        self.window_size = self.default_window_size
        self.linear_stationary_threshold_m_s = float(linear_stationary_threshold_m_s)
        self.linear_motion_threshold_m_s = float(linear_motion_threshold_m_s)
        self.angular_stationary_threshold_rad_s = float(angular_stationary_threshold_rad_s)
        self.angular_motion_threshold_rad_s = float(angular_motion_threshold_rad_s)
        self.imu_yaw_rate_stationary_threshold_rad_s = float(imu_yaw_rate_stationary_threshold_rad_s)
        self.lidar_static_confidence_threshold = float(lidar_static_confidence_threshold)
        self.predictive_index_threshold = float(predictive_index_threshold)
        self.spawn_anchor = SpawnAnchorAccumulator(
            sample_count=spawn_anchor_sample_count,
            snap_step_m=spawn_snap_step_m,
        )

        self.predictive_history: deque[float] = deque(maxlen=self.window_size)
        self.block_history: deque[float] = deque(maxlen=self.window_size)
        self.pose_x: float | None = None
        self.pose_y: float | None = None
        self.pose_yaw: float | None = None
        self.spawn_x: float | None = None
        self.spawn_y: float | None = None
        self.previous_stamp_s: float | None = None
        self.previous_yaw: float | None = None
        self.gps_anchor_pending = True

    def _resize_histories(self) -> None:
        self.predictive_history = deque(list(self.predictive_history)[-self.window_size :], maxlen=self.window_size)
        self.block_history = deque(list(self.block_history)[-self.window_size :], maxlen=self.window_size)

    def set_window_size(self, size: int) -> None:
        self.window_size = max(1, int(size))
        self._resize_histories()

    def reset(self, size: int | None = None) -> None:
        self.window_size = self.default_window_size if size is None else max(1, int(size))
        self.predictive_history = deque(maxlen=self.window_size)
        self.block_history = deque(maxlen=self.window_size)
        self.pose_x = None
        self.pose_y = None
        self.pose_yaw = None
        self.spawn_x = None
        self.spawn_y = None
        self.previous_stamp_s = None
        self.previous_yaw = None
        self.spawn_anchor.reset()
        self.gps_anchor_pending = True

    @property
    def anchor_sample_count(self) -> int:
        return self.spawn_anchor.count

    @property
    def anchor_sample_target(self) -> int:
        return self.spawn_anchor.sample_count

    def pending_anchor_pose(self, fallback_x: float, fallback_y: float) -> tuple[float, float]:
        buffered = self.spawn_anchor.buffered_pose((fallback_x, fallback_y))
        return buffered if buffered is not None else (fallback_x, fallback_y)

    def _anchor_from_gps(self, raw_x: float, raw_y: float, spawn_x: float, spawn_y: float, yaw: float, stamp_s: float) -> PoseEstimate:
        self.pose_x = spawn_x
        self.pose_y = spawn_y
        self.pose_yaw = yaw
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.previous_stamp_s = stamp_s
        self.previous_yaw = yaw
        self.predictive_history.clear()
        self.block_history.clear()
        self.gps_anchor_pending = False
        return PoseEstimate(
            x=spawn_x,
            y=spawn_y,
            yaw=yaw,
            dt_s=0.0,
            wheel_linear_velocity=0.0,
            wheel_angular_velocity=0.0,
            imu_yaw_rate_rad_s=0.0,
            predictive_index=0.0,
            predictive_sample=0.0,
            block_index=0.0,
            lidar_motion_index=0.0,
            translation_distance_m=0.0,
            translation_enabled=False,
            translation_frozen=True,
            stationary=True,
            rotating_in_place=False,
            slip_likely=False,
            gps_anchor_used=True,
            gps_anchor_pending=False,
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            raw_x=raw_x,
            raw_y=raw_y,
            window_size=self.window_size,
        )

    def update(
        self,
        *,
        raw_x: float,
        raw_y: float,
        yaw: float,
        stamp_s: float,
        wheel_motion: WheelMotionEstimate,
        lidar_motion: LidarMotionEstimate,
    ) -> PoseEstimate | None:
        if self.pose_x is None or self.pose_y is None or self.pose_yaw is None or self.gps_anchor_pending:
            snapped_spawn = self.spawn_anchor.add_sample(raw_x, raw_y)
            self.previous_stamp_s = stamp_s
            self.previous_yaw = yaw
            if snapped_spawn is None:
                return None
            return self._anchor_from_gps(
                snapped_spawn.raw_x,
                snapped_spawn.raw_y,
                snapped_spawn.snapped_x,
                snapped_spawn.snapped_y,
                yaw,
                stamp_s,
            )

        dt = self.expected_dt_s
        if self.previous_stamp_s is not None and stamp_s > self.previous_stamp_s:
            dt = stamp_s - self.previous_stamp_s
        self.previous_stamp_s = stamp_s

        imu_yaw_rate = 0.0
        if self.previous_yaw is not None and dt > 1e-6:
            imu_yaw_rate = angular_delta(yaw, self.previous_yaw) / dt
        self.previous_yaw = yaw

        linear_velocity = float(wheel_motion.linear_velocity)
        angular_velocity = float(wheel_motion.angular_velocity)
        linear_speed = abs(linear_velocity)
        angular_speed = abs(angular_velocity)
        imu_yaw_speed = abs(imu_yaw_rate)
        lidar_motion_index = clamp(1.0 - float(lidar_motion.stationary_confidence), 0.0, 1.0)

        linear_motion_index = clamp(
            (linear_speed - self.linear_stationary_threshold_m_s)
            / max(self.linear_motion_threshold_m_s - self.linear_stationary_threshold_m_s, 1e-6),
            0.0,
            1.0,
        )
        rotation_index = clamp(
            (max(angular_speed, imu_yaw_speed) - self.angular_stationary_threshold_rad_s)
            / max(self.angular_motion_threshold_rad_s - self.angular_stationary_threshold_rad_s, 1e-6),
            0.0,
            1.0,
        )

        stationary = (
            linear_speed <= self.linear_stationary_threshold_m_s
            and angular_speed <= self.angular_stationary_threshold_rad_s
            and imu_yaw_speed <= self.imu_yaw_rate_stationary_threshold_rad_s
            and float(lidar_motion.stationary_confidence) >= self.lidar_static_confidence_threshold
        )
        rotating_in_place = (
            linear_speed <= self.linear_motion_threshold_m_s
            and max(angular_speed, imu_yaw_speed) >= self.angular_stationary_threshold_rad_s
        )
        slip_likely = (
            linear_speed > self.linear_motion_threshold_m_s
            and float(lidar_motion.stationary_confidence) >= self.lidar_static_confidence_threshold
        )

        spin_penalty = rotation_index * clamp(1.0 - linear_motion_index, 0.0, 1.0)
        predictive_sample = clamp(linear_motion_index * lidar_motion_index * (1.0 - spin_penalty), 0.0, 1.0)
        block_sample = 1.0 if stationary or rotating_in_place or slip_likely else 0.0
        self.predictive_history.append(predictive_sample)
        self.block_history.append(block_sample)
        predictive_index = mean_or_zero(self.predictive_history)
        block_index = mean_or_zero(self.block_history)

        translation_frozen = bool(stationary or rotating_in_place or slip_likely)
        translation_enabled = bool(
            linear_speed > self.linear_stationary_threshold_m_s
            and not rotating_in_place
            and not slip_likely
        )
        translation_distance = linear_velocity * dt if translation_enabled else 0.0

        forward_x, forward_y = forward_vector_from_yaw(yaw)
        self.pose_x += translation_distance * forward_x
        self.pose_y += translation_distance * forward_y
        self.pose_yaw = yaw

        return PoseEstimate(
            x=float(self.pose_x),
            y=float(self.pose_y),
            yaw=float(yaw),
            dt_s=float(dt),
            wheel_linear_velocity=linear_velocity,
            wheel_angular_velocity=angular_velocity,
            imu_yaw_rate_rad_s=float(imu_yaw_rate),
            predictive_index=float(predictive_index),
            predictive_sample=float(predictive_sample),
            block_index=float(block_index),
            lidar_motion_index=float(lidar_motion_index),
            translation_distance_m=float(translation_distance),
            translation_enabled=translation_enabled,
            translation_frozen=translation_frozen,
            stationary=stationary,
            rotating_in_place=rotating_in_place,
            slip_likely=slip_likely,
            gps_anchor_used=False,
            gps_anchor_pending=self.gps_anchor_pending,
            spawn_x=float(self.spawn_x if self.spawn_x is not None else raw_x),
            spawn_y=float(self.spawn_y if self.spawn_y is not None else raw_y),
            raw_x=raw_x,
            raw_y=raw_y,
            window_size=self.window_size,
        )
