from devices import Robot, Device
import controller


class DistanceSensor(Device):
    device: controller.DistanceSensor

    def __init__(self, robot: Robot, name: str, time_step: int | None = None, offset=None):
        super().__init__(offset)
        self.device = robot.get_device(name)
        if time_step is None:
            time_step = robot.get_time_step()
        self.device.enable(time_step)

    def get_distance(self):
        return self.device.getValue()
