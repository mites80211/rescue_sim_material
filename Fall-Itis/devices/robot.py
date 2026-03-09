import controller
import re
import sys


class Robot:
    device: controller.Robot

    def __init__(self, time_step: int | None = None):
        self.device = controller.Robot()
        if time_step is None:
            time_step = int(self.device.getBasicTimeStep())
        self.time_step = time_step

    @staticmethod
    def _canonical_device_name(name: str) -> str:
        token_aliases = {
            'dist': 'distance',
            'colour': 'color',
            'pos': 'position',
        }
        tokens = re.sub(r'[^a-z0-9]+', ' ', name.lower()).split()
        return ' '.join(token_aliases.get(token, token) for token in tokens)

    def _available_device_names(self) -> list[str]:
        devices = getattr(self.device, 'devices', None)
        if isinstance(devices, dict):
            return list(devices.keys())
        return []

    def _get_known_device(self, name: str):
        devices = getattr(self.device, 'devices', None)
        if isinstance(devices, dict):
            return devices.get(name)
        return self.device.getDevice(name)

    def get_device(self, name):
        device = self._get_known_device(name)
        if device is not None:
            return device

        canonical_name = self._canonical_device_name(name)
        matches = [
            candidate for candidate in self._available_device_names()
            if self._canonical_device_name(candidate) == canonical_name
        ]
        if len(matches) == 1:
            resolved_name = matches[0]
            print(
                f'Warning: using "{resolved_name}" for requested device "{name}".',
                file=sys.stderr
            )
            return self._get_known_device(resolved_name)
        if len(matches) > 1:
            raise ValueError(
                f'Device "{name}" is ambiguous. Matching devices: {", ".join(matches)}'
            )

        available = self._available_device_names()
        available_label = ', '.join(available) if available else '<unavailable>'
        raise ValueError(
            f'Device "{name}" was not found. Available devices: {available_label}'
        )

    def step(self, time_step=None):
        return self.device.step(self.time_step if time_step is None else time_step)

    def get_time_step(self):
        return self.time_step
