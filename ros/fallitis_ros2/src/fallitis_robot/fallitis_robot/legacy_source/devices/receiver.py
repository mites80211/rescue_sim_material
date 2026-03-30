from devices import Robot, Device
import controller


class Receiver(Device):
    device: controller.Receiver

    def __init__(self, robot: Robot, name: str, time_step: int | None = None, offset=None):
        super().__init__(offset)
        self.device = robot.get_device(name)
        if time_step is None:
            time_step = robot.get_time_step()
        self.device.enable(time_step)

    def get_queue_length(self):
        return self.device.getQueueLength()

    def get_bytes(self):
        return self.device.getBytes()

    def next_packet(self):
        self.device.nextPacket()
