import socket
import struct


class UdpSender:
    """阻塞式 UDP 发送器，send() 直接调用 sendto。"""

    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, msg_type: int, timestamp_ns: int, payload: bytes):
        """阻塞发送，直到数据被内核接收。"""
        header = struct.pack("!IQI", msg_type, timestamp_ns, len(payload))
        self.sock.sendto(header + payload, self.addr)
