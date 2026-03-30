import struct
from functools import partial


class Supervisor:

    class Events:
        lack_of_progress = 'lack_of_progress'
        game_information = 'game_information'

    def __init__(self, emitter, receiver):
        self.emitter = emitter
        self.receiver = receiver
        self.listeners = {self.Events.lack_of_progress: [], self.Events.game_information: []}

    def add_listener(self, event, action):
        if event not in self.listeners:
            raise KeyError(f'Invalid event: {event}')
        self.listeners[event].append(action)

    def handle_received_data(self):
        while self.receiver.get_queue_length() > 0:
            data = self.receiver.get_bytes()
            packet_type = chr(data[0])
            if packet_type == 'G':
                self.handle_game_information(data)
            elif packet_type == 'L':
                self.handle_lack_of_progress()
            self.receiver.next_packet()

    def handle_game_information(self, data):
        packet_format = 'c f i'
        packet_size = struct.calcsize(packet_format)
        tup = struct.unpack(packet_format, data[:packet_size])
        self._call_listeners(self.Events.game_information, tup[1], tup[2])

    def handle_lack_of_progress(self):
        self._call_listeners(self.Events.lack_of_progress)

    def _call_listeners(self, event, *args):
        listeners = self.listeners[event]
        for listener in listeners:
            if len(args) > 0:
                listener = partial(listener, *args)
            listener()

    def score_game_element(self, sign_type, x, z):
        packet_format = 'i i c'
        packet_data = struct.pack(packet_format, int(x * 100), int(z * 100), bytes(sign_type, 'utf-8'))
        self.emitter.send(packet_data)

    def call_lack_of_progress(self):
        packet_data = struct.pack('c', b'L')
        self.emitter.send(packet_data)

    def call_end_of_play(self):
        packet_data = struct.pack('c', b'E')
        self.emitter.send(packet_data)

    def request_game_information(self):
        packet_data = struct.pack('c', b'G')
        self.emitter.send(packet_data)

    def submit_map_bonus(self, matrix):
        matrix = matrix.astype(str)
        shape_data = struct.pack('2i', *matrix.shape)
        matrix_data = ','.join(matrix.flatten()).encode('utf-8')
        self.emitter.send(shape_data + matrix_data)
        self.emitter.send(struct.pack('c', b'M'))
