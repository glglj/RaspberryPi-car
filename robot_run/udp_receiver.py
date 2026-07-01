import socket
import struct
import threading
import time
from model.models import (
    MotionCommand, LoopClosure, PoseCorrection,
    CMD_STOP, CMD_STRAIGHT, CMD_TURN_LEFT, CMD_TURN_RIGHT,
    MSG_CMD_REPLY, MSG_LOOP_CLOSURE, MSG_POSE_CORRECTION,
)


class TcpReceiver:
    """TCP 服务端: 监听 PC 指令 (PC → Pi)

    后台 accept 线程，新连接替换旧连接。
    支持:
    - 运动指令 (兼容旧5字节格式 + 新header格式)
    - SLAM回环约束 (MSG_LOOP_CLOSURE)
    - 位姿修正 (MSG_POSE_CORRECTION)
    """

    def __init__(self, port=5006):
        self.port = port
        self._client_sock = None
        self._lock = threading.Lock()
        self._running = True

        # 创建 server socket
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind(("0.0.0.0", port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(1.0)

        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True, name="tcp-recv-accept")
        self._accept_thread.start()

    # ---- 后台 accept 循环 ----
    def _accept_loop(self):
        """持续 accept，新连接替换旧连接"""
        while self._running:
            try:
                conn, addr = self._server_sock.accept()
                print(f"[TcpReceiver] PC 已连接: {addr}")
                with self._lock:
                    old = self._client_sock
                    self._client_sock = conn
                if old:
                    try:
                        old.close()
                    except OSError:
                        pass
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.5)

    # ---- 精确读取 n 字节 ----
    @staticmethod
    def _recv_exactly(sock, n):
        """从 socket 精确读取 n 字节，失败返回 None"""
        buf = bytearray()
        while len(buf) < n:
            try:
                chunk = sock.recv(n - len(buf))
                if not chunk:
                    return None  # 连接关闭
                buf += chunk
            except socket.timeout:
                return None
            except OSError:
                return None
        return bytes(buf)

    # ---- 接收一帧 ----
    def recv(self, timeout=0.1):
        """返回 (type, data) 元组或 None

        type: 'motion', 'loop_closure', 'pose_correction'
        data: MotionCommand | LoopClosure | PoseCorrection
        """
        with self._lock:
            sock = self._client_sock
        if sock is None:
            return None

        sock.settimeout(timeout)

        try:
            # 读取 12 字节 header
            header = self._recv_exactly(sock, 12)
            if header is None:
                return None

            msg_type, ts, payload_len = struct.unpack("!IQI", header)

            # 读取 payload
            payload = self._recv_exactly(sock, payload_len)
            if payload is None:
                return None

            if msg_type == MSG_CMD_REPLY and len(payload) >= 5:
                cmd_type, param = struct.unpack("<Bf", payload[:5])
                if cmd_type in (CMD_STOP, CMD_STRAIGHT,
                               CMD_TURN_LEFT, CMD_TURN_RIGHT):
                    return ('motion', MotionCommand(cmd_type=cmd_type, param=param))

            elif msg_type == MSG_LOOP_CLOSURE and len(payload) >= 24:
                loop = LoopClosure.unpack(payload[:24])
                return ('loop_closure', loop)

            elif msg_type == MSG_POSE_CORRECTION and len(payload) >= 16:
                corr = PoseCorrection.unpack(payload[:16])
                return ('pose_correction', corr)

        except socket.timeout:
            pass
        except OSError:
            pass
        return None

    def close(self):
        self._running = False
        with self._lock:
            if self._client_sock:
                try:
                    self._client_sock.close()
                except OSError:
                    pass
                self._client_sock = None
        try:
            self._server_sock.close()
        except OSError:
            pass
