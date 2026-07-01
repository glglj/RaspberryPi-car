import socket
import struct
from model.models import (
    MotionCommand, LoopClosure, PoseCorrection,
    CMD_STOP, CMD_STRAIGHT, CMD_TURN_LEFT, CMD_TURN_RIGHT,
    MSG_CMD, MSG_CMD_REPLY, MSG_LOOP_CLOSURE, MSG_POSE_CORRECTION,
)


class UdpReceiver:
    """UDP 指令监听，上位机 → Pi

    支持:
    - 运动指令 (兼容旧5字节格式 + 新header格式)
    - SLAM回环约束 (MSG_LOOP_CLOSURE)
    - 位姿修正 (MSG_POSE_CORRECTION)
    """

    def __init__(self, port=5006):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.1)

    def recv(self, timeout=0.1):
        """返回 (type, data) 元组或 None

        type: 'motion', 'loop_closure', 'pose_correction'
        data: MotionCommand | LoopClosure | PoseCorrection
        """
        self.sock.settimeout(timeout)
        try:
            raw, addr = self.sock.recvfrom(65536)
            if len(raw) < 5:
                return None

            # 尝试新格式: | msg_type(uint32 BE) | timestamp_ns(uint64 BE) | payload_len(uint32 BE) | payload |
            if len(raw) >= 16:
                msg_type, ts, payload_len = struct.unpack("!IQI", raw[:16])
                payload = raw[16:16 + payload_len]

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

            # 兼容旧格式: | cmd_type(uint8 LE) | param(float32 LE) | = 5 bytes
            if len(raw) >= 5:
                cmd_type, param = struct.unpack("<Bf", raw[:5])
                if cmd_type in (CMD_STOP, CMD_STRAIGHT,
                               CMD_TURN_LEFT, CMD_TURN_RIGHT):
                    return ('motion', MotionCommand(cmd_type=cmd_type, param=param))

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