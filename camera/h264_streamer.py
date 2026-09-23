"""
树莓派端 H264 视频推流器

从 USB 摄像头读取 H264 原生码流，通过 TCP 推送到 PC 端。
不重编码（-c copy），实现低延迟传输。

使用方式:
    python camera/h264_streamer.py --host 192.168.1.100 --port 5007
    python camera/h264_streamer.py --host bj.zyfrp.vip --port 5007  # 通过 frp
"""

import subprocess
import socket
import threading
import time
import argparse
import os


class H264Streamer:
    """H264 摄像头推流器

    - subprocess 启动 ffmpeg 从 /dev/video0 读取 H264，输出到 pipe
    - 后台线程从 ffmpeg stdout 读取 → TCP socket.sendall()
    - 断线自动重连（指数退避），重连时重启 ffmpeg 进程
    - 每秒打印统计（码率、发送帧数）
    """

    def __init__(self, host, port,
                 device="/dev/video0", width=1280, height=720, fps=30):
        self.host = host
        self.port = port
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps

        self._running = False
        self._ffmpeg_proc = None
        self._sock = None

        # 统计
        self._stats_lock = threading.Lock()
        self._bytes_sent = 0
        self._last_stats_time = time.time()

    # ---- ffmpeg 进程 ----
    def _start_ffmpeg(self):
        """启动 ffmpeg 子进程，从摄像头读取 H264 输出到 stdout pipe"""
        cmd = [
            "ffmpeg",
            "-f", "v4l2",
            "-input_format", "h264",
            "-video_size", f"{self.width}x{self.height}",
            "-framerate", str(self.fps),
            "-i", self.device,
            "-c", "copy",              # 透传，零延迟
            "-f", "h264",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-avioflags", "direct",
            "-flush_packets", "1",
            "pipe:1"
        ]
        self._ffmpeg_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        print(f"[H264Streamer] ffmpeg 已启动: {' '.join(cmd)}")

    def _stop_ffmpeg(self):
        """停止 ffmpeg 进程"""
        proc = self._ffmpeg_proc
        if proc is not None:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    proc.kill()
                    proc.wait(timeout=2)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        self._ffmpeg_proc = None

    # ---- TCP 连接和发送 ----
    def _connect_and_stream(self):
        """建立 TCP 连接并持续发送 H264 数据"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(5.0)
        sock.connect((self.host, self.port))
        sock.settimeout(None)
        self._sock = sock
        print(f"[H264Streamer] 已连接到 {self.host}:{self.port}")

        # 启动 ffmpeg
        self._start_ffmpeg()
        stdout = self._ffmpeg_proc.stdout

        # 循环读取 ffmpeg 输出并发送
        chunk_size = 32768  # 32KB 块
        while self._running:
            try:
                data = stdout.read(chunk_size)
            except (OSError, ValueError):
                break

            if not data:
                break  # ffmpeg 进程退出

            try:
                sock.sendall(data)
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

            with self._stats_lock:
                self._bytes_sent += len(data)

        self._stop_ffmpeg()

    def _close_sock(self):
        """关闭当前 socket"""
        sock = self._sock
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        self._sock = None

    # ---- 主循环 ----
    def _run_loop(self):
        """主循环：自动重连 + 指数退避"""
        backoff = 1.0
        while self._running:
            try:
                self._connect_and_stream()
            except (OSError, ConnectionRefusedError, socket.timeout):
                pass
            finally:
                self._close_sock()
                self._stop_ffmpeg()

            if not self._running:
                break

            print(f"[H264Streamer] 连接断开，{backoff:.0f}s 后重连...")
            time.sleep(min(backoff, 16.0))
            backoff = min(backoff * 2, 16.0)

        print("[H264Streamer] 推流线程退出")

    # ---- 统计线程 ----
    def _stats_loop(self):
        """每秒打印统计信息"""
        while self._running:
            time.sleep(1.0)
            now = time.time()
            with self._stats_lock:
                elapsed = now - self._last_stats_time
                if elapsed <= 0:
                    continue
                bytes_sent = self._bytes_sent
                self._bytes_sent = 0
                self._last_stats_time = now

            kbps = (bytes_sent * 8) / (elapsed * 1000)
            print(f"[H264Streamer] 码率={kbps:.0f} kbps, "
                  f"数据={bytes_sent/1024:.1f} KB/s")

    # ---- 公开接口 ----
    def start(self):
        """启动推流（非阻塞）"""
        self._running = True
        self._stream_thread = threading.Thread(
            target=self._run_loop, daemon=True, name="h264-stream")
        self._stats_thread = threading.Thread(
            target=self._stats_loop, daemon=True, name="h264-stats")
        self._stream_thread.start()
        self._stats_thread.start()
        print(f"[H264Streamer] 推流已启动 → {self.host}:{self.port} "
              f"({self.width}x{self.height} @{self.fps}fps)")

    def stop(self):
        """停止推流"""
        self._running = False
        self._close_sock()
        self._stop_ffmpeg()
        print("[H264Streamer] 推流已停止")


def main():
    parser = argparse.ArgumentParser(description="H264 视频推流器")
    parser.add_argument("--host", default="127.0.0.1", help="TCP 目标地址")
    parser.add_argument("--port", type=int, default=5007, help="TCP 目标端口")
    parser.add_argument("--device", default="/dev/video0", help="摄像头设备路径")
    parser.add_argument("--width", type=int, default=1280, help="分辨率宽")
    parser.add_argument("--height", type=int, default=720, help="分辨率高")
    parser.add_argument("--fps", type=int, default=30, help="帧率")
    args = parser.parse_args()

    streamer = H264Streamer(
        host=args.host,
        port=args.port,
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    streamer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        streamer.stop()


if __name__ == "__main__":
    main()
