import pigpio
import struct

class EncoderSensor:
    def __init__(self, pi, a_pins=(17, 27), b_pins=(22, 10)):
        self.pi = pi
        self.a_cb = [pi.callback(pin, pigpio.EITHER_EDGE) for pin in a_pins if pin is not None]
        self.b_cb = [pi.callback(pin, pigpio.EITHER_EDGE) for pin in b_pins if pin is not None]

        self.last_a = sum(cb.tally() for cb in self.a_cb)
        self.last_b = sum(cb.tally() for cb in self.b_cb)

    def get_edges(self):
        """返回自上次调用以来的增量"""
        a_now = sum(cb.tally() for cb in self.a_cb)
        b_now = sum(cb.tally() for cb in self.b_cb)

        a_edges = a_now - self.last_a
        b_edges = b_now - self.last_b

        self.last_a = a_now
        self.last_b = b_now

        return a_edges, b_edges

    def get_payload(self):
        """直接给 UDP 用"""
        a_edges, b_edges = self.get_edges()
        return struct.pack('<II', a_edges, b_edges)
