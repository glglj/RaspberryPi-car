import socket
import struct
import time

class UDPSender:
    def __init__(self, ip, port, local_port=6000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", local_port))      # ⭐ 必须
        self.sock.settimeout(0.02)            # ⭐ 非阻塞

        self.addr = (ip, port)
        self.queue = []

    def send_payload(self, msg_type, payload=b''):
        header = struct.pack('<HHI', 0xAA55, msg_type, len(payload))
        packet = header + payload
        self.sock.sendto(packet, self.addr)

    def send(self, msg_type, payload=b''):
        self.send_payload(msg_type, payload)

    def recv(self):
        try:
            data, _ = self.sock.recvfrom(2048)
            return data
        except socket.timeout:
            return None

    def loop_send(self, lidar_sensor, encoder_sensor):
        print("UDP线程启动")

        while True:
            # ===== 1. 雷达 =====
            lidar_sensor.update()
            frame = lidar_sensor.get_frame()

            if frame:
                lidar_payload = b''.join(
                    struct.pack('<ff', angle, dist)
                    for angle, dist in frame
                )
                self.send(0x01, lidar_payload)

            # ===== 2. 编码器 =====
            a_edges, b_edges = encoder_sensor.get_edges()
            encoder_payload = struct.pack('<II', a_edges, b_edges)
            self.send(0x02, encoder_payload)

            # ===== 3. 拉指令 =====
            self.send(0x10)

            data = self.recv()
            if data:
                magic, msg_type, length = struct.unpack('<HHI', data[:8])
                payload = data[8:8 + length]

                if magic == 0xAA55 and msg_type == 0x11:
                    cmd = payload.decode()
                    print("[CMD]", cmd)

                    # 👉 简化版（先别跨文件）
                    if cmd == "PING":
                        pass

                    self.send(0x12, cmd.encode())

            time.sleep(0.05)

    def add_sensor(self, msg_type, sensor):
        self.queue.append((msg_type, sensor))
