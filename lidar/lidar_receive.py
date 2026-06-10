import time
import struct
import serial
import threading
import queue
from lidar.lidar_parser import LidarParser


class LidarSensor:
    """激光雷达传感器驱动，后台线程持续读取，凑满一圈存入队列供 UDP 发送。

    用法:
        lidar = LidarSensor()
        # 后台线程自动开始采集
        frame = lidar.get_frame()  # 非阻塞，返回一圈数据或 None
    """

    def __init__(self, port="/dev/serial0", baudrate=230400):
        self.parser = LidarParser()
        self.uart = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self.frame_queue = queue.Queue(maxsize=1)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        """后台线程：持续从串口读取并解析雷达数据，凑满一圈入队。"""
        current = []
        while self._running:
            data = self.uart.read(4096)
            if not data:
                continue
            pkgs = self.parser.feed(data)
            for pkg in pkgs:
                # is_start 表示新一圈开始，提交上一圈
                if pkg.get("is_start") and current:
                    # DEBUG: 检查 frame 内角度是否单调递增
                    angles = [p[0] for p in current]
                    min_a, max_a = min(angles), max(angles)
                    drops = sum(1 for i in range(1, len(angles)) if angles[i] < angles[i-1])

                    self._enqueue(current)

                    current = []
                for si in pkg["Si"]:
                    current.append((si["angle"], si["distance"]))


    def _enqueue(self, frame):
        """将完整的一圈数据加入队列，队列满时丢弃最旧的一圈。"""
        ts = time.time_ns()
        if self.frame_queue.full():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self.frame_queue.put_nowait((ts, frame))
        except queue.Full:
            pass

    def get_frame(self, block=True, timeout=None):
        """获取最近的一圈完整雷达数据。

        Returns:
            (timestamp, list of (angle, distance)) 或 None（超时时）
        """
        try:
            return self.frame_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    @staticmethod
    def pack_frame(ts, frame):
        """将一圈雷达数据打包为单个 UDP payload（位压缩，不分片）。

        每点 20 bit 存 3 字节:
          - 角度索引 9 bit (角度/0.9°, 精度 0.9°, 范围 0~459°)
          - 距离      11 bit (距离/10mm, 精度 10mm, 范围 0~20.47m)
        载荷格式: | ts (uint64, 8B) | N × 3B packed points |
        """
        buf = bytearray(8 + len(frame) * 3)
        struct.pack_into("<Q", buf, 0, ts)
        offset = 8
        for angle, distance in frame:
            ai = min(511, int(angle / 0.9 + 0.5))      # 9 bits
            ds = min(2047, int(distance / 10 + 0.5))   # 11 bits
            packed = (ai << 11) | ds                    # 20 bits
            buf[offset:offset + 3] = packed.to_bytes(3, 'little')
            offset += 3
        return bytes(buf)

    def update(self):
        """供主循环周期性调用，兼容旧接口。后台线程已处理所有读取工作。"""
        pass

    def stop(self):
        """停止后台读取线程。"""
        self._running = False


if __name__ == "__main__":
    import time
    lidar = LidarSensor()
    print("雷达后台线程已启动，等待数据...")
    try:
        while True:
            frame = lidar.get_frame()
            if frame:
                print(f"收到一圈: {len(frame)} 个点, ")
            time.sleep(0.01)
    except KeyboardInterrupt:
        lidar.stop()
        print("退出")