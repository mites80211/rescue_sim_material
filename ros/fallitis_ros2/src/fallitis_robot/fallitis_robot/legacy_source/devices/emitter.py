from devices import Robot, Device
import controller


class Emitter(Device):
    device: controller.Emitter

    def __init__(self, robot: Robot, name: str, offset=None):
        super().__init__(offset)
        self.device = robot.get_device(name)

    def send(self, message):
        self.device.send(message)
