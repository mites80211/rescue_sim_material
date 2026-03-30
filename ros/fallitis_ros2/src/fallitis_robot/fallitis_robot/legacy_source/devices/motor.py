from devices import Robot, Device
import controller


class Motor(Device):
    device: controller.Motor

    def __init__(self, robot: Robot, name: str, offset=None):
        super().__init__(offset)
        self.device = robot.get_device(name)
        self.device.setPosition(float('+inf'))
        self.device.setVelocity(0.0)

    def get_velocity(self):
        return self.device.getVelocity()

    def set_velocity(self, velocity: float):
        self.device.setVelocity(velocity)
