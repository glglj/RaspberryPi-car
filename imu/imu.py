import serial
import threading
import time
import struct
from queue import Queue
from imu.imu_parser import ImuParser
import math


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
    def __init__(self, port="/dev/ttyUSB0", baud=115200, queue_size=10):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.parser = ImuParser()
        self.queue = Queue(maxsize=queue_size)
        self.running = False
        self.thread = None
        self._bundle_timeout = 0.015
        self._last_data_ns = 0
        self._lock = threading.Lock()
        self._latest_yaw = 0.0           # 最新偏航角 (度)
        self._latest_gyro_z = 0.0        # 最新Z轴角速度 (度/秒)
        self._latest_bundle = (0, None)  # 最新IMU二进制包

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

    @property
    def latest_yaw(self):
        """返回最新的偏航角 (度)"""
        with self._lock:
            return self._latest_yaw

    @property
    def latest_gyro_z(self):
        """返回最新的Z轴角速度 (度/秒)"""
        with self._lock:
            return self._latest_gyro_z

    def get_latest_bundle(self):
        """返回最新的IMU二进制包 (非消费读取)，供统一发送线程使用。"""
        with self._lock:
            return self._latest_bundle

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
                        self._extract_imu_state(partial)
                        self._put(time.time_ns(), _pack_bundle(partial))
                time.sleep(0.001)  # 防止无数据时空转 CPU
                continue

            self._last_data_ns = time.time_ns()
            for bundle in self.parser.feed(data):
                self._extract_imu_state(bundle)
                self._put(time.time_ns(), _pack_bundle(bundle))

    def _extract_imu_state(self, bundle):
        """从解析后的bundle中提取yaw和gyro_z，供里程计和运动控制使用"""
        for f in bundle:
            t = f["type"]
            if t == 0x53:  # ANGLE
                with self._lock:
                    self._latest_yaw = f["yaw"]
            elif t == 0x52:  # GYRO
                with self._lock:
                    self._latest_gyro_z = f["gz"]

    def _put(self, ts, payload):
        if self.queue.full():
            try:
                self.queue.get_nowait()
            except Exception:
                pass
        self.queue.put((ts, payload))
        with self._lock:
            self._latest_bundle = (ts, payload)