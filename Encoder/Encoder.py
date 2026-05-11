import time
import threading
from queue import Queue

import pigpio

from dataclasses import dataclass


@dataclass
class EncoderFrame:
    timestamp: float
    a_edges: int
    b_edges: int


class EdgeCounter:
    def __init__(self, pi, pins):
        self.callbacks = [
            pi.callback(pin, pigpio.EITHER_EDGE)
            for pin in pins
            if pin is not None
        ]

    def total(self):
        return sum(cb.tally() for cb in self.callbacks)


class EncoderSensor:
    def __init__(
        self,
        pi,
        a_pins=(17, 27),
        b_pins=(22, 10),
        sample_rate=100,
        queue_size=1000
    ):
        self.pi = pi

        self.a_counter = EdgeCounter(pi, a_pins)
        self.b_counter = EdgeCounter(pi, b_pins)

        self.last_a = self.a_counter.total()
        self.last_b = self.b_counter.total()

        self.queue = Queue(maxsize=queue_size)

        self.sample_interval = 1.0 / sample_rate

        self.running = False
        self.thread = None

    # =========================
    # 读取一次
    # =========================

    def read(self):
        a_now = self.a_counter.total()
        b_now = self.b_counter.total()

        frame = EncoderFrame(
            timestamp=time.time(),
            a_edges=a_now - self.last_a,
            b_edges=b_now - self.last_b
        )

        self.last_a = a_now
        self.last_b = b_now

        return frame

    # =========================
    # 后台线程
    # =========================

    def _worker(self):
        while self.running:
            frame = self.read()

            if not self.queue.full():
                self.queue.put(frame)

            time.sleep(self.sample_interval)

    # =========================
    # 启动
    # =========================

    def start(self):
        if self.running:
            return

        self.running = True

        self.thread = threading.Thread(
            target=self._worker,
            daemon=True
        )

        self.thread.start()

    # =========================
    # 停止
    # =========================

    def stop(self):
        self.running = False

        if self.thread:
            self.thread.join()

    # =========================
    # 获取数据
    # =========================

    def get(self, block=True, timeout=None):
        return self.queue.get(
            block=block,
            timeout=timeout
        )