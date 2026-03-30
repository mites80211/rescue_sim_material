from devices import Robot, Device
import controller


class InertialUnit(Device):
    device: controller.InertialUnit

    def __init__(self, robot: Robot, name: str, time_step: int | None = None, offset=None):
        super().__init__(offset)
        self.device = robot.get_device(name)
        if time_step is None:
            time_step = robot.get_time_step()
        self.device.enable(time_step)

    def get_rpy(self):
        return self.device.getRollPitchYaw()

    def get_roll(self):
        return self.get_rpy()[0]

    def get_pitch(self):
        return self.get_rpy()[1]

    def get_yaw(self):
        return self.get_rpy()[2]
