import serial
import threading
import time
import struct
from queue import Queue
from model.models import (
    IMUAccel, IMUGyro, IMUAngle, IMUMag, IMUQuat
)
from imu.imu_parser import ImuParser

# dict → dataclass 映射
_TYPE_MAP = {
    0x51: lambda d: IMUAccel(d["ax"], d["ay"], d["az"], d["temp"]),
    0x52: lambda d: IMUGyro(d["gx"], d["gy"], d["gz"]),
    0x53: lambda d: IMUAngle(d["roll"], d["pitch"], d["yaw"]),
    0x54: lambda d: IMUMag(d["mx"], d["my"], d["mz"]),
    0x59: lambda d: IMUQuat(d["q0"], d["q1"], d["q2"], d["q3"]),
}


class IMUSensor:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, queue_size=100):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.parser = ImuParser()
        self.queue = Queue(maxsize=queue_size)
        self.running = False
        self.thread = None

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

    @staticmethod
    def pack(item):
        if isinstance(item, IMUAccel):
            return struct.pack("<BBffff", 0x51, 4, item.ax, item.ay, item.az, item.temp)
        elif isinstance(item, IMUGyro):
            return struct.pack("<BBfff", 0x52, 3, item.gx, item.gy, item.gz)
        elif isinstance(item, IMUAngle):
            return struct.pack("<BBfff", 0x53, 3, item.roll, item.pitch, item.yaw)
        elif isinstance(item, IMUMag):
            return struct.pack("<BBfff", 0x54, 3, item.mx, item.my, item.mz)
        elif isinstance(item, IMUQuat):
            return struct.pack("<BBffff", 0x59, 4, item.q0, item.q1, item.q2, item.q3)
        return b""

    # =========================
    # 线程（Cython 接管所有字节级解析）
    # =========================
    def _worker(self):
        while self.running:
            data = self.ser.read(64)
            if not data:
                continue
            frames = self.parser.feed(data)
            for f in frames:
                make = _TYPE_MAP.get(f["type"])
                if make is None:
                    continue
                obj = make(f)
                ts = time.time_ns()
                if self.queue.full():
                    try:
                        self.queue.get_nowait()
                    except Exception:
                        pass
                self.queue.put((ts, obj))