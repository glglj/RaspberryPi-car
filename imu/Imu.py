import serial
import threading
import time
import struct
from queue import Queue
from dataclasses import dataclass
from model.models import (
    IMUAccel, IMUGyro, IMUAngle, IMUMag, IMUQuat
)


class IMUSensor:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, queue_size=100):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.buffer = bytearray()

        self.queue = Queue(maxsize=queue_size)

        self.running = False
        self.thread = None

        self.last_group_time = None

        # parser注册表
        self.parsers = {
            0x51: self._parse_acc,
            0x52: self._parse_gyro,
            0x53: self._parse_angle,
            0x54: self._parse_mag,
            0x59: self._parse_quat,
        }

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
    # 线程
    # =========================
    def _worker(self):
        while self.running:
            self.buffer += self.ser.read(64)
            self._parse_buffer()

    # =========================
    # 帧解析
    # =========================
    def _parse_buffer(self):
        while len(self.buffer) >= 11:

            # 帧头
            if self.buffer[0] != 0x55:
                self.buffer.pop(0)
                continue

            dtype = self.buffer[1]

            if not (0x50 <= dtype <= 0x5F):
                self.buffer.pop(0)
                continue

            frame = self.buffer[:11]
            self.buffer = self.buffer[11:]

            self._handle_frame(frame)

    # =========================
    # 分发
    # =========================
    def _handle_frame(self, frame):
        dtype = frame[1]
        data = frame[2:10]
        crc = frame[10]

        # CRC
        calc_crc = sum(frame[:10]) & 0xFF
        if calc_crc != crc:
            return

        # 时间统计（只在0x51触发）
        if dtype == 0x51:
            now = time.time()
            if self.last_group_time:
                dt = (now - self.last_group_time) * 1000
                # 你可以关掉print
                print(f"📦 周期: {dt:.2f} ms")
            self.last_group_time = now

        parser = self.parsers.get(dtype)
        if parser:
            obj = parser(data)
            if obj:
                if self.queue.full():
                    try:
                        self.queue.get_nowait()
                    except:
                        pass
                self.queue.put(obj)

    # =========================
    # 小端解析工具
    # =========================
    def _u16(self, l, h):
        return struct.unpack('<h', bytes([l, h]))[0]

    # =========================
    # 各类解析
    # =========================
    def _parse_acc(self, data):
        ax = self._u16(data[0], data[1]) / 32768 * 16
        ay = self._u16(data[2], data[3]) / 32768 * 16
        az = self._u16(data[4], data[5]) / 32768 * 16
        temp = self._u16(data[6], data[7]) / 100

        return IMUAccel(ax, ay, az, temp)

    def _parse_gyro(self, data):
        gx = self._u16(data[0], data[1]) / 32768 * 2000
        gy = self._u16(data[2], data[3]) / 32768 * 2000
        gz = self._u16(data[4], data[5]) / 32768 * 2000

        return IMUGyro(gx, gy, gz)

    def _parse_angle(self, data):
        roll  = self._u16(data[0], data[1]) / 32768 * 180
        pitch = self._u16(data[2], data[3]) / 32768 * 180
        yaw   = self._u16(data[4], data[5]) / 32768 * 180

        return IMUAngle(roll, pitch, yaw)

    def _parse_mag(self, data):
        mx = self._u16(data[0], data[1])
        my = self._u16(data[2], data[3])
        mz = self._u16(data[4], data[5])

        return IMUMag(mx, my, mz)

    def _parse_quat(self, data):
        q0 = self._u16(data[0], data[1]) / 32768
        q1 = self._u16(data[2], data[3]) / 32768
        q2 = self._u16(data[4], data[5]) / 32768
        q3 = self._u16(data[6], data[7]) / 32768

        return IMUQuat(q0, q1, q2, q3)