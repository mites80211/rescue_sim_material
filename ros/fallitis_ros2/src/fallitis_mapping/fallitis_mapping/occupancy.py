from __future__ import annotations

import math

import cv2
import numpy as np


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OccupancyGridModel:
    unknown, free, occupied = range(3)

    def __init__(self, shape: tuple[int, int], spacing: int, origin: tuple[float, float]) -> None:
        self.shape = shape
        self.spacing = spacing
        shape_half = [value // 2 for value in shape]
        origin_scaled = [round(value * self.spacing) for value in origin]
        self.offset = (shape_half[0] - origin_scaled[0], shape_half[1] + origin_scaled[1])
        self.occupancy = np.full(self.shape, self.unknown, dtype=np.uint8)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        return (
            round(x * self.spacing + self.offset[0]),
            round(-y * self.spacing + self.offset[1]),
        )

    def grid_to_world(self, x: int, y: int) -> tuple[float, float]:
        return (
            (x - self.offset[0]) / self.spacing,
            -(y - self.offset[1]) / self.spacing,
        )

    def update_region(self, grid: np.ndarray, center: tuple[int, int]) -> np.ndarray | None:
        half_h, half_w = grid.shape[0] // 2, grid.shape[1] // 2
        x0 = center[0] - half_h
        x1 = center[0] + half_h
        y0 = center[1] - half_w
        y1 = center[1] + half_w
        if x0 < 0 or y0 < 0 or x1 > self.shape[0] or y1 > self.shape[1]:
            return None
        region = self.occupancy[x0:x1, y0:y1]
        np.putmask(region, grid > region, grid)
        return region

    def occupancy_data(self) -> np.ndarray:
        data = np.full(self.shape, -1, dtype=np.int16)
        data[self.occupancy == self.free] = 0
        data[self.occupancy == self.occupied] = 100
        return data


class LidarDataProcessor:
    def __init__(
        self,
        *,
        max_range_m: float,
        hor_res: int,
        hor_fov: float,
        ver_res: int,
        ver_fov: float,
        yaw_offset_rad: float,
        occupancy_grid: OccupancyGridModel,
    ) -> None:
        self.max_range_m = max_range_m
        self.hor_res = hor_res
        self.hor_fov = hor_fov
        self.ver_res = ver_res
        self.ver_fov = ver_fov
        self.yaw_offset_rad = yaw_offset_rad
        self.occupancy_grid = occupancy_grid

        self.first_lyr = ver_fov / 2.0
        self.ver_angle = ver_fov / max(1, ver_res - 1)
        self.max_radius = round(max_range_m * occupancy_grid.spacing)
        self.polar_shape = (hor_res, self.max_radius)
        self.polar_x = np.arange(self.polar_shape[0])
        self.polar_y = np.arange(self.polar_shape[1])
        self.theta = np.sqrt(
            1
            - np.square(
                np.sin(self.first_lyr - self.ver_angle * np.arange(ver_res)[:, np.newaxis])
            )
        )
        self.flags = cv2.WARP_INVERSE_MAP + cv2.WARP_POLAR_LINEAR + cv2.WARP_FILL_OUTLIERS
        self.warp_shape = (self.polar_shape[1] * 2, self.polar_shape[1] * 2)
        self.warp_center = (self.polar_shape[1], self.polar_shape[1])

    def lidar_scan(self, range_image: np.ndarray, yaw: float) -> np.ndarray:
        image = np.copy(range_image)
        image[~np.isfinite(image)] = float("+inf")
        image = image * self.theta
        image = np.rint(image * self.occupancy_grid.spacing).astype(int)
        polar = np.full(self.polar_shape, self.occupancy_grid.unknown, dtype=np.uint8)
        for layer in image:
            mask = (layer > 0) & (layer < self.polar_shape[1])
            polar[self.polar_x[mask], layer[mask]] = self.occupancy_grid.occupied
            polar[self.polar_y < layer[:, None]] = self.occupancy_grid.free
        mask = (image <= 0) | (image >= self.polar_shape[1])
        polar[self.polar_x[np.all(mask, axis=0)], :] = self.occupancy_grid.free
        shift = round(math.degrees(yaw + self.yaw_offset_rad) * self.polar_shape[0] / 360.0)
        polar = np.roll(polar, -shift, axis=0)
        return cv2.warpPolar(
            polar,
            self.warp_shape,
            self.warp_center,
            self.max_radius,
            self.flags,
        )
