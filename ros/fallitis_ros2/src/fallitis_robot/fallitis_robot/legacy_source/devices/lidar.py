import numpy as np

from devices import Robot, Device
import controller


class Lidar(Device):
    device: controller.Lidar

    def __init__(self, robot: Robot, name: str, time_step: int | None = None, max_range=None, offset=None):
        super().__init__(offset)
        self.device = robot.get_device(name)
        if time_step is None:
            time_step = robot.get_time_step()
        self.device.enable(time_step)

        self.max_range = self.device.getMaxRange() if max_range is None else max_range
        self.hor_res = self.device.getHorizontalResolution()
        self.hor_fov = self.device.getFov()
        self.ver_res = self.device.getNumberOfLayers()
        self.ver_fov = self.device.getVerticalFov()

    def get_range_image(self):
        im = np.copy(self.device.getRangeImage())
        im[im > self.max_range] = float('+inf')
        return im.reshape(self.ver_res, self.hor_res)

    def set_max_range(self, value):
        self.max_range = value
