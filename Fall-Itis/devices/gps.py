from devices import Robot, Device
import controller


class GPS(Device):
    device: controller.GPS

    def __init__(self, robot: Robot, name: str, time_step: int | None = None, offset=None):
        super().__init__(offset)
        self.device = robot.get_device(name)
        if time_step is None:
            time_step = robot.get_time_step()
        self.device.enable(time_step)

    def get_xyz(self):
        return self.device.getValues()

    def get_x(self):
        return self.get_xyz()[0]

    def get_y(self):
        return self.get_xyz()[1]

    def get_z(self):
        return self.get_xyz()[2]
