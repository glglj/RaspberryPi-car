import pigpio
from lidar.lidar_to_PC import LidarSensor
from Encoder_to_PC import EncoderSensor
from udp_sender import UDPSender
import time
import threading
import subprocess
import struct

# ------------------ 消息类型 ------------------
MSG_LIDAR      = 0x01
MSG_ENCODER   = 0x02
MSG_CMD_PULL  = 0x10
MSG_CMD_REPLY = 0x11
MSG_CMD_ACK   = 0x12

# ------------------ pigpio 启动 ------------------
def start_pigpio():
    pi = pigpio.pi()
    if pi.connected:
        print("pigpio already running")
        return pi
    pi.stop()

    subprocess.run(["sudo", "killall", "pigpiod"],
                   stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)

    print("Starting pigpio daemon...")
    subprocess.run(["sudo", "pigpiod", "-l"])

    for _ in range(10):
        pi = pigpio.pi()
        if pi.connected:
            print("pigpio daemon connected")
            return pi
        time.sleep(0.2)

    raise RuntimeError("Failed to connect to pigpio")

# ------------------ 指令处理 ------------------
def handle_cmd(cmd: str, udp_sender):
    print(f"[CMD] 收到指令: {cmd}")

    if cmd == "PING":
        pass

    elif cmd == "STOP_LIDAR":
        print("停止雷达（示例）")
        # lidar.stop()

    elif cmd.startswith("SET_FREQ"):
        freq = int(cmd.split()[1])
        print("设置频率:", freq)

    else:
        print("未知指令")

    # 执行完成 ACK
    udp_sender.send(MSG_CMD_ACK, cmd.encode())

# ------------------ UDP 主循环 ------------------///
    while True:
        LidarSensor.update()
        frame = LidarSensor.get_frame()

        if frame:
            lidar_payload = b''.join(
                struct.pack('<ff', angle, dist)
                for angle, dist in frame
            )

            a_edges, b_edges = EncoderSensor.get_edges()
            encoder_payload = struct.pack('<II', a_edges, b_edges)

            udp_sender.send(MSG_LIDAR, lidar_payload)
            udp_sender.send(MSG_ENCODER, encoder_payload)

        # -------- 2. 拉取指令 --------
        udp_sender.send(MSG_CMD_PULL)

        # -------- 3. 接收服务器回包 --------
        data = udp_sender.recv()
        if data:
            magic, msg_type, length = struct.unpack('<HHI', data[:8])
            payload = data[8:8+length]

            if magic == 0xAA55 and msg_type == MSG_CMD_REPLY and payload:
                cmd = payload.decode()
                handle_cmd(cmd, udp_sender)

        time.sleep(0.05)   # 20Hz 拉指令

# ------------------ 主程序 ------------------
def main():
    pi = start_pigpio()

    lidar = LidarSensor()
    encoder = EncoderSensor(pi)

    udp_sender = UDPSender("bj.zyfrp.vip", 5005)

    threading.Thread(
        target=udp_sender.loop_send,
        args=( lidar, encoder),
        daemon=True
    ).start()

    print("系统运行中")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pi.stop()
        print("退出程序")

if __name__ == "__main__":
    main()
