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
        self.frame_queue = queue.Queue(maxsize=50)
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
    def iter_payloads(ts, frame, chunk_size=180):
        """将一圈雷达帧拆成 MTU 安全的分片 payload。

        Yields:
            (chunk_ts, payload_bytes) 可直接传给 UdpSender.send()
        """
        total = (len(frame) + chunk_size - 1) // chunk_size
        for idx in range(total):
            chunk = frame[idx * chunk_size : (idx + 1) * chunk_size]
            points_data = b"".join(
                struct.pack("<ff", angle, dist) for angle, dist in chunk
            )
            payload = struct.pack("<QBB", ts, idx, total) + points_data
            yield ts, payload

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