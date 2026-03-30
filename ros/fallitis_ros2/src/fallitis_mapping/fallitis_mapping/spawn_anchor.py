from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import statistics


@dataclass(slots=True)
class SnappedSpawn:
    raw_x: float
    raw_y: float
    snapped_x: float
    snapped_y: float


class SpawnAnchorAccumulator:
    def __init__(self, *, sample_count: int, snap_step_m: float) -> None:
        self.sample_count = max(1, int(sample_count))
        self.snap_step_m = float(snap_step_m)
        self.samples_x: deque[float] = deque(maxlen=self.sample_count)
        self.samples_y: deque[float] = deque(maxlen=self.sample_count)

    @property
    def count(self) -> int:
        return len(self.samples_x)

    def reset(self) -> None:
        self.samples_x.clear()
        self.samples_y.clear()

    def buffered_pose(self, fallback: tuple[float, float] | None = None) -> tuple[float, float] | None:
        if not self.samples_x or not self.samples_y:
            return fallback
        return (
            float(statistics.median(self.samples_x)),
            float(statistics.median(self.samples_y)),
        )

    def add_sample(self, raw_x: float, raw_y: float) -> SnappedSpawn | None:
        self.samples_x.append(float(raw_x))
        self.samples_y.append(float(raw_y))
        buffered = self.buffered_pose()
        if buffered is None or self.count < self.sample_count:
            return None
        snapped_x = self.snap_axis(buffered[0])
        snapped_y = self.snap_axis(buffered[1])
        return SnappedSpawn(
            raw_x=float(buffered[0]),
            raw_y=float(buffered[1]),
            snapped_x=snapped_x,
            snapped_y=snapped_y,
        )

    def snap_axis(self, value: float) -> float:
        if self.snap_step_m <= 0.0:
            return float(value)
        snapped_index = round(value / self.snap_step_m)
        return float(snapped_index * self.snap_step_m)
