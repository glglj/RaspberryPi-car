import serial
import struct
from lidar.lidar_parser import LidarParser

class LidarSensor:
    def __init__(self, port="/dev/serial0", baudrate=230400):
        self.parser = LidarParser()
        self.uart = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self.points = []

    def read_from_uart(self):
        """读取 UART 数据并解析"""
        data = self.uart.read(4096)
        if not data:
            return []
        pkgs = self.parser.feed(data)
        return pkgs

    def update(self):
        """获取雷达数据并累积到 points"""
        pkgs = self.read_from_uart()
        for pkg in pkgs:
            for si in pkg["Si"]:
                angle = si["angle"]
                dist = si["distance"]
                self.points.append((angle, dist))

    def get_frame(self):
        """判断是否凑够一圈，如果是返回完整点云并清空 points"""
        if not self.points:
            return None
        angles = [p[0] for p in self.points]
        max_angle = max(angles)
        min_angle = min(angles)
        angle_span = max_angle - min_angle if max_angle - min_angle >= 0 else max_angle - min_angle + 360
        if angle_span >= 358:  # 凑够一圈
            frame = self.points.copy()
            self.points.clear()
            return frame
        return None

    @staticmethod
    def get_payload(frame):
        """生成 UDP payload（frame 是完整一圈点）"""
        payload = b''.join([struct.pack('<ff', angle, dist) for angle, dist in frame])
        return payload
