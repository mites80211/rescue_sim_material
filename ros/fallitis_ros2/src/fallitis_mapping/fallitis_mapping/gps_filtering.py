from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from sensor_msgs.msg import JointState, LaserScan


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass(slots=True)
class WheelMotionEstimate:
    left_rad_s: float = 0.0
    right_rad_s: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    stamp_s: float = 0.0


@dataclass(slots=True)
class LidarMotionEstimate:
    stationary_confidence: float = 0.0
    median_delta_m: float = float("inf")
    directional_motion_delta_m: float = 0.0
    close_point_count: int = 0
    points_xy: list[tuple[float, float]] = None  # type: ignore[assignment]
    directional_ranges: dict[str, float | None] = None  # type: ignore[assignment]
    directional_deltas: dict[str, float | None] = None  # type: ignore[assignment]
    range_max_m: float = 0.0

    def __post_init__(self) -> None:
        if self.points_xy is None:
            self.points_xy = []
        if self.directional_ranges is None:
            self.directional_ranges = {}
        if self.directional_deltas is None:
            self.directional_deltas = {}


class WheelOdometryTracker:
    def __init__(self, *, wheel_radius: float, axle_length: float) -> None:
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length
        self.latest = WheelMotionEstimate()

    def update(self, msg: JointState) -> WheelMotionEstimate:
        lookup = {name: idx for idx, name in enumerate(msg.name)}
        left_idx = lookup.get("left_wheel")
        right_idx = lookup.get("right_wheel")
        left = float(msg.velocity[left_idx]) if left_idx is not None and left_idx < len(msg.velocity) else 0.0
        right = float(msg.velocity[right_idx]) if right_idx is not None and right_idx < len(msg.velocity) else 0.0
        stamp_s = stamp_to_seconds(msg.header.stamp)
        linear = self.wheel_radius * (left + right) * 0.5
        angular = self.wheel_radius * (right - left) / self.axle_length
        self.latest = WheelMotionEstimate(
            left_rad_s=left,
            right_rad_s=right,
            linear_velocity=linear,
            angular_velocity=angular,
            stamp_s=stamp_s,
        )
        return self.latest


class LidarMotionTracker:
    DIRECTION_ANGLES = {
        "front": 0.0,
        "front_right": -math.pi / 4,
        "right": -math.pi / 2,
        "rear_right": -3 * math.pi / 4,
        "rear": math.pi,
        "rear_left": 3 * math.pi / 4,
        "left": math.pi / 2,
        "front_left": math.pi / 4,
    }

    def __init__(
        self,
        *,
        close_range_m: float,
        stationary_range_eps_m: float,
        downsample: int,
    ) -> None:
        self.close_range_m = close_range_m
        self.stationary_range_eps_m = stationary_range_eps_m
        self.downsample = max(1, int(downsample))
        self.previous_ranges: np.ndarray | None = None

    def reset(self) -> None:
        self.previous_ranges = None

    def update(self, msg: LaserScan, predicted_linear_velocity: float) -> LidarMotionEstimate:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        finite = np.isfinite(ranges)
        directional_ranges = self.directional_ranges(msg, ranges)
        sample_indices = np.arange(0, len(ranges), self.downsample)
        points_xy: list[tuple[float, float]] = []
        for idx in sample_indices:
            value = ranges[idx]
            if not np.isfinite(value) or value > self.close_range_m:
                continue
            angle = msg.angle_min + idx * msg.angle_increment
            points_xy.append((float(value * math.cos(angle)), float(value * math.sin(angle))))

        median_delta = float("inf")
        directional_deltas: dict[str, float | None] = {label: None for label in self.DIRECTION_ANGLES}
        directional_motion_delta = 0.0
        if self.previous_ranges is not None and len(self.previous_ranges) == len(ranges):
            previous = self.previous_ranges
            close_mask = finite & np.isfinite(previous) & ((ranges < self.close_range_m) | (previous < self.close_range_m))
            if np.any(close_mask):
                deltas = np.abs(ranges[close_mask] - previous[close_mask])
                median_delta = float(np.median(deltas))
            previous_directional = self.directional_ranges(msg, previous)
            valid_directional_deltas: list[float] = []
            for label, current_value in directional_ranges.items():
                previous_value = previous_directional.get(label)
                if current_value is None or previous_value is None:
                    directional_deltas[label] = None
                    continue
                delta = abs(float(current_value) - float(previous_value))
                directional_deltas[label] = float(delta)
                if label in {"front", "rear", "left", "right"}:
                    valid_directional_deltas.append(delta)
            if valid_directional_deltas:
                directional_motion_delta = float(max(valid_directional_deltas))
        self.previous_ranges = ranges.copy()

        stationary_confidence = 0.0
        motion_reference = directional_motion_delta if directional_motion_delta > 0.0 else median_delta
        if np.isfinite(motion_reference):
            normalized = 1.0 - motion_reference / max(self.stationary_range_eps_m, 1e-6)
            stationary_confidence = clamp(normalized, 0.0, 1.0)
        if abs(predicted_linear_velocity) < 0.002:
            stationary_confidence = max(stationary_confidence, 0.5)

        return LidarMotionEstimate(
            stationary_confidence=stationary_confidence,
            median_delta_m=median_delta,
            directional_motion_delta_m=directional_motion_delta,
            close_point_count=len(points_xy),
            points_xy=points_xy,
            directional_ranges=directional_ranges,
            directional_deltas=directional_deltas,
            range_max_m=float(msg.range_max),
        )

    def directional_ranges(self, msg: LaserScan, ranges: np.ndarray) -> dict[str, float | None]:
        values: dict[str, float | None] = {}
        for label, target_angle in self.DIRECTION_ANGLES.items():
            index = round((target_angle - msg.angle_min) / msg.angle_increment)
            if label == "rear":
                index = max(0, min(len(ranges) - 1, index))
                if index == len(ranges):
                    index = len(ranges) - 1
            if index < 0 or index >= len(ranges):
                values[label] = None
                continue
            value = ranges[index]
            values[label] = float(value) if np.isfinite(value) else None
        return values
