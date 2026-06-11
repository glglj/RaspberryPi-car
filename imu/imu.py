import serial
import threading
import time
import struct
from queue import Queue
from imu.imu_parser import ImuParser


def _pack_frame(f):
    """单帧 dict → 二进制: | type(1B) | data(N*4B) |"""
    t = f["type"]
    if t == 0x51:      # ACC: 4 floats
        return struct.pack("<Bffff", 0x51, f["ax"], f["ay"], f["az"], f["temp"])
    elif t == 0x52:    # GYRO: 3 floats
        return struct.pack("<Bfff", 0x52, f["gx"], f["gy"], f["gz"])
    elif t == 0x53:    # ANGLE: 3 floats
        return struct.pack("<Bfff", 0x53, f["roll"], f["pitch"], f["yaw"])
    elif t == 0x54:    # MAG: 3 int16 (as float)
        return struct.pack("<Bfff", 0x54, f["mx"], f["my"], f["mz"])
    elif t == 0x59:    # QUAT: 4 floats
        return struct.pack("<Bffff", 0x59, f["q0"], f["q1"], f["q2"], f["q3"])
    return b""


def _pack_bundle(frames):
    """帧列表 → 二进制: | count(1B) | frame1 | frame2 | ... |"""
    parts = [struct.pack("<B", len(frames))]
    for f in frames:
        parts.append(_pack_frame(f))
    return b"".join(parts)


class IMUSensor:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, queue_size=100):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.parser = ImuParser()
        self.queue = Queue(maxsize=queue_size)
        self.running = False
        self.thread = None
        self._bundle_timeout = 0.015
        self._last_data_ns = 0

    # =========================
    # 外部接口
    # =========================
    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def get(self, block=True, timeout=None):
        return self.queue.get(block=block, timeout=timeout)

    # =========================
    # 工作线程：串口读取 → Cython 解包 → 直接打包二进制 → 入队
    # =========================
    def _worker(self):
        while self.running:
            data = self.ser.read(64)
            if not data:
                # 超时兜底：长时间无数据，取出残余帧
                if self._last_data_ns and (
                    time.time_ns() - self._last_data_ns > self._bundle_timeout * 1e9
                ):
                    partial = self.parser.flush()
                    if partial:
                        self._put(time.time_ns(), _pack_bundle(partial))
                continue

            self._last_data_ns = time.time_ns()
            for bundle in self.parser.feed(data):
                self._put(time.time_ns(), _pack_bundle(bundle))

    def _put(self, ts, payload):
        if self.queue.full():
            try:
                self.queue.get_nowait()
            except Exception:
                pass
        self.queue.put((ts, payload))