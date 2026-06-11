import socket
import struct
from model.models import MotionCommand, CMD_STOP, CMD_STRAIGHT, CMD_TURN_LEFT, CMD_TURN_RIGHT


class UdpReceiver:
    """UDP 指令监听，上位机 → Pi"""

    def __init__(self, port=5006):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.1)

    def recv(self, timeout=0.1) -> MotionCommand | None:
        """返回解析后的指令，无数据时返回 None"""
        self.sock.settimeout(timeout)
        try:
            data, addr = self.sock.recvfrom(1024)
            if len(data) >= 5:
                cmd_type, param = struct.unpack("<Bf", data[:5])
                if cmd_type in (CMD_STOP, CMD_STRAIGHT, CMD_TURN_LEFT, CMD_TURN_RIGHT):
                    return MotionCommand(cmd_type=cmd_type, param=param)
        except socket.timeout:
            pass
        except OSError:
            pass
        return None

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
