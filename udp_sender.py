import socket
import struct
import threading
import time


class TcpSender:
    """TCP 客户端: 连接 PC 并发送数据 (Pi → PC)

    - 后台线程自动重连 (指数退避 1s→2s→4s→max 16s)
    - send() 失败时标记断连，后台线程自动恢复
    - 线程安全: _lock 保护 sock/_connected
    """

    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = None
        self._connected = False
        self._lock = threading.Lock()
        self._running = True

        self.last_call_ns = None
        self.max_send_us = 0

        self._reconnect_thread = threading.Thread(
            target=self._connect_loop, daemon=True, name="tcp-sender-reconnect")
        self._reconnect_thread.start()

    # ---- 后台重连 ----
    def _connect_loop(self):
        """指数退避连接，连接成功后等待断连信号"""
        backoff = 1.0
        while self._running:
            with self._lock:
                if self._connected:
                    # 已连接，等待 send() 检测到断连
                    pass
                else:
                    # 尝试连接
                    try:
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(5.0)
                        s.connect(self.addr)
                        s.settimeout(None)
                        self.sock = s
                        self._connected = True
                        backoff = 1.0
                        print(f"[TcpSender] 已连接到 {self.addr}")
                    except OSError:
                        if self.sock:
                            try:
                                self.sock.close()
                            except OSError:
                                pass
                        self.sock = None
            # 未连接时等待重试
            if not self._connected and self._running:
                time.sleep(min(backoff, 16.0))
                backoff = min(backoff * 2, 16.0)
            else:
                time.sleep(0.5)

    # ---- 发送 ----
    def send(self, msg_type, timestamp_ns, payload):
        """发送一帧数据，返回 True/False"""
        now_ns = time.perf_counter_ns()
        self.last_call_ns = now_ns

        header = struct.pack("!IQI", msg_type, timestamp_ns, len(payload))
        data = header + payload

        with self._lock:
            if not self._connected or self.sock is None:
                return False
            sock = self.sock

        t0 = time.perf_counter_ns()
        try:
            sock.sendall(data)
        except (BrokenPipeError, ConnectionResetError, OSError):
            with self._lock:
                self._connected = False
                self.sock = None
            try:
                sock.close()
            except OSError:
                pass
            return False

        t1 = time.perf_counter_ns()
        send_us = (t1 - t0) / 1000
        if send_us > self.max_send_us:
            self.max_send_us = send_us
        return True

    def close(self):
        self._running = False
        with self._lock:
            self._connected = False
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
