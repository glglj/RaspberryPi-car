import socket
import struct
import time


class UdpSender:

    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, msg_type, timestamp_ns, payload):
        header = struct.pack(
            "!IQI",
            msg_type,
            timestamp_ns,
            len(payload)
        )
        self.sock.sendto(
            header + payload,
            self.addr
        )
