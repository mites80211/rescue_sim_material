import cv2
import numpy as np

from devices import Robot, Device
import controller


class Camera(Device):
    device: controller.Camera

    def __init__(self, robot: Robot, name: str, time_step: int | None = None, offset=None):
        super().__init__(offset)
        self.device = robot.get_device(name)
        if time_step is None:
            time_step = robot.get_time_step()
        self.device.enable(time_step)

        self.width = self.device.getWidth()
        self.height = self.device.getHeight()
        self.shape = np.array((self.height, self.width, 4))

    def get_image(self):
        return np.array(np.frombuffer(self.device.getImage(), np.uint8).reshape(self.shape))
