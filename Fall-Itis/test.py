import controller
import struct

robot = controller.Robot()
emitter = controller.Emitter("emitter")
for _ in range(64):
    robot.step(time_step=16)
for _ in range(4):
    robot.step(time_step=16)


packet_format = 'i i c'
packet_data = struct.pack(packet_format, int(-6), int(-11), bytes('O', 'utf-8'))
emitter.send(packet_data)
robot.step(time_step=16)
robot.step(time_step=16)
