import serial
import time
from lidar_parser import LidarParser


class LidarPacketDebug:
    def __init__(self, port="/dev/serial0", baudrate=230400):
        self.parser = LidarParser()
        self.uart = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)

    def read_uart(self):
        data = self.uart.read(4096)

        if not data:
            return []
        return self.parser.feed(data)

    def run(self):
        while True:
            pkgs = self.read_uart()

            for pkg in pkgs:

                # =========================
                # 时间戳（关键）
                # =========================
                ts = time.time_ns()

                angles = []

                # =========================
                # 提取角度
                # =========================
                for si in pkg["Si"]:
                    angles.append(si["angle"])

                if len(angles) == 0:
                    continue

                # =========================
                # FSA / LSA（协议字段）
                # =========================
                fsa = pkg.get("FSA", None)
                lsa = pkg.get("LSA", None)
                lsn = pkg.get("LSN", len(angles))

                min_angle = min(angles)
                max_angle = max(angles)

                # 角度跨度（处理360°回绕）
                span = max_angle - min_angle
                if span < 0:
                    span += 360

                # =========================
                # 输出调试信息
                # =========================
                print("\n==============================")
                print(f"timestamp (ns): {ts}")
                print(f"FSA: {fsa}")
                print(f"LSA: {lsa}")
                print(f"LSN: {lsn}")
                print(f"min_angle: {min_angle:.2f}")
                print(f"max_angle: {max_angle:.2f}")
                print(f"span: {span:.2f}")
                print(f"points: {len(angles)}")
                print("==============================\n")


if __name__ == "__main__":
    lidar = LidarPacketDebug()
    lidar.run()