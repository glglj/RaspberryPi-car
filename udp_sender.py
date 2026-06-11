import socket
import struct
import time


class UdpSender:

    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.last_call_ns = None
        self.max_send_us = 0

    def send(self, msg_type, timestamp_ns, payload):

        now_ns = time.perf_counter_ns()

        # 检查距离上次调用间隔
       


        self.last_call_ns = now_ns

        header = struct.pack(
            "!IQI",
            msg_type,
            timestamp_ns,
            len(payload)
        )

        t0 = time.perf_counter_ns()

        self.sock.sendto(
            header + payload,
            self.addr
        )

        t1 = time.perf_counter_ns()

        send_us = (t1 - t0) / 1000

        if send_us > self.max_send_us:
            self.max_send_us = send_us

