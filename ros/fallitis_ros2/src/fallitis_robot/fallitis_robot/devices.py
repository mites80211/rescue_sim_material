from __future__ import annotations

import struct
import re
import sys
from dataclasses import dataclass

import numpy as np

try:
    import controller
except ImportError as exc:  # pragma: no cover - only available inside Webots.
    controller = None
    CONTROLLER_IMPORT_ERROR = exc
else:
    CONTROLLER_IMPORT_ERROR = None


def require_controller() -> None:
    if controller is None:
        raise RuntimeError(
            "The Webots 'controller' module is not available. "
            "Run this node from a Webots controller environment."
        ) from CONTROLLER_IMPORT_ERROR


@dataclass(slots=True)
class DeviceOffset:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float64)


class RobotDriver:
    token_aliases = {
        "dist": "distance",
        "colour": "color",
        "pos": "position",
    }

    def __init__(self, time_step: int | None = None) -> None:
        require_controller()
        self.device = controller.Robot()
        if time_step is None:
            time_step = int(self.device.getBasicTimeStep())
        self.time_step = time_step

    @classmethod
    def canonical_name(cls, name: str) -> str:
        tokens = re.sub(r"[^a-z0-9]+", " ", name.lower()).split()
        return " ".join(cls.token_aliases.get(token, token) for token in tokens)

    def available_device_names(self) -> list[str]:
        devices = getattr(self.device, "devices", None)
        if isinstance(devices, dict):
            return list(devices.keys())
        return []

    def _get_known_device(self, name: str):
        devices = getattr(self.device, "devices", None)
        if isinstance(devices, dict):
            return devices.get(name)
        return self.device.getDevice(name)

    def get_device(self, name: str):
        device = self._get_known_device(name)
        if device is not None:
            return device

        canonical_name = self.canonical_name(name)
        matches = [
            candidate
            for candidate in self.available_device_names()
            if self.canonical_name(candidate) == canonical_name
        ]
        if len(matches) == 1:
            resolved_name = matches[0]
            print(
                f'Warning: using "{resolved_name}" for requested device "{name}".',
                file=sys.stderr,
            )
            return self._get_known_device(resolved_name)
        if len(matches) > 1:
            raise ValueError(
                f'Device "{name}" is ambiguous. Matching devices: {", ".join(matches)}'
            )

        available = self.available_device_names()
        available_label = ", ".join(available) if available else "<unavailable>"
        raise ValueError(
            f'Device "{name}" was not found. Available devices: {available_label}'
        )

    def step(self, time_step: int | None = None) -> int:
        return self.device.step(self.time_step if time_step is None else time_step)


class MotorDevice:
    def __init__(self, robot: RobotDriver, name: str) -> None:
        self.device = robot.get_device(name)
        self.device.setPosition(float("inf"))
        self.device.setVelocity(0.0)

    def set_velocity(self, velocity: float) -> None:
        self.device.setVelocity(float(velocity))

    def get_velocity(self) -> float:
        return float(self.device.getVelocity())


class EmitterDevice:
    def __init__(self, robot: RobotDriver, name: str) -> None:
        self.device = robot.get_device(name)

    def send_bytes(self, payload: bytes) -> None:
        self.device.send(payload)

    def send_letter(self, letter: str) -> None:
        if len(letter) != 1:
            raise ValueError("EmitterDevice.send_letter expects a single character.")
        self.send_bytes(struct.pack("c", letter.encode("utf-8")))


class GpsDevice:
    def __init__(self, robot: RobotDriver, name: str, time_step: int | None = None) -> None:
        self.device = robot.get_device(name)
        self.device.enable(robot.time_step if time_step is None else time_step)

    def xyz(self) -> np.ndarray:
        return np.asarray(self.device.getValues(), dtype=np.float64)


class InertialUnitDevice:
    def __init__(self, robot: RobotDriver, name: str, time_step: int | None = None) -> None:
        self.device = robot.get_device(name)
        self.device.enable(robot.time_step if time_step is None else time_step)

    def rpy(self) -> np.ndarray:
        return np.asarray(self.device.getRollPitchYaw(), dtype=np.float64)


class CameraDevice:
    def __init__(self, robot: RobotDriver, name: str, time_step: int | None = None) -> None:
        self.device = robot.get_device(name)
        self.device.enable(robot.time_step if time_step is None else time_step)
        self.width = int(self.device.getWidth())
        self.height = int(self.device.getHeight())

    def image(self) -> np.ndarray:
        raw = self.device.getImage()
        return np.frombuffer(raw, np.uint8).reshape((self.height, self.width, 4)).copy()


class LidarDevice:
    def __init__(self, robot: RobotDriver, name: str, time_step: int | None = None) -> None:
        self.device = robot.get_device(name)
        self.device.enable(robot.time_step if time_step is None else time_step)
        self.max_range = float(self.device.getMaxRange())
        self.hor_res = int(self.device.getHorizontalResolution())
        self.hor_fov = float(self.device.getFov())
        self.ver_res = int(self.device.getNumberOfLayers())
        self.ver_fov = float(self.device.getVerticalFov())

    def range_image(self) -> np.ndarray:
        data = np.asarray(self.device.getRangeImage(), dtype=np.float32).copy()
        image = data.reshape((self.ver_res, self.hor_res))
        image[image <= 0.0] = np.nan
        image[image > self.max_range] = np.nan
        return image
