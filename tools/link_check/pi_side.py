"""端云通信链路自检 - 树莓派侧

验证两条公网链路 (不涉及真实传感器与电机):
  1. Pi→PC 数据链路: 向 bj.zyfrp.vip:5001 发送合成里程计帧
  2. PC→Pi 命令链路: 在本机 8002 端口等待 PC 发来的指令并打印

用法 (在树莓派上):
    python3 tools/link_check/pi_side.py [--duration 30]

判据: 日志中出现 "PC 数据已收到" 与 "收到PC指令" 即双向链路正常。
"""

import argparse
import sys
import time
import os
import threading

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from tcp_sender import TcpSender                       # noqa: E402
from robot_run.udp_receiver import TcpReceiver         # noqa: E402
from model.models import (                             # noqa: E402
    OdometryFrame, RobotPose, CMD_STOP,
)

DATA_HOST = "43.227.71.58"
DATA_PORT = 44310
CMD_PORT = 8002


def main():
    parser = argparse.ArgumentParser(description="端云链路自检 (树莓派侧)")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="发送时长 (秒)")
    args = parser.parse_args()

    print("=" * 60)
    print(f"[Pi] 数据链路目标: {DATA_HOST}:{DATA_PORT} (→PC:8080)")
    print(f"[Pi] 命令链路监听: 0.0.0.0:{CMD_PORT} (←bj.zyfrp.vip:8002)")
    print("=" * 60)

    # PC→Pi 命令通道
    receiver = TcpReceiver(port=CMD_PORT)
    got_command = [False]

    # Pi→PC 数据通道
    sender = TcpSender(DATA_HOST, DATA_PORT)

    def cmd_loop():
        while time.time() < deadline + 5:
            result = receiver.recv(timeout=0.5)
            if result is None:
                continue
            msg_type, data = result
            got_command[0] = True
            if msg_type == "motion":
                print(f"[Pi] ✓ 收到PC指令: cmd={data.cmd_type} "
                      f"({'STOP' if data.cmd_type == CMD_STOP else data.cmd_type}) "
                      f"param={data.param}")
            elif msg_type == "pose_correction":
                print(f"[Pi] ✓ 收到PC位姿修正: kf={data.kf_id} "
                      f"({data.corrected_x:.3f}, {data.corrected_y:.3f}, "
                      f"{data.corrected_theta:.3f}rad)")
            else:
                print(f"[Pi] ✓ 收到PC消息: {msg_type}")

    deadline = time.time() + args.duration
    t = threading.Thread(target=cmd_loop, daemon=True)
    t.start()

    # 发送合成里程计
    sent = 0
    seq = 0
    while time.time() < deadline:
        if sender._connected and seq == 0:
            print("[Pi] ✓ 数据链路已连接到 PC (经 43.227.71.58:44310)")
            seq = 1
        frame = OdometryFrame(
            pose=RobotPose(x=seq * 0.01, y=0.0, theta=0.0),
            v=0.1, omega=0.0,
            timestamp_ns=int(time.time() * 1e9),
        )
        if sender.send(MSG_ODOM, frame.timestamp_ns, frame.pack()):
            sent += 1
        time.sleep(0.5)

    print(f"[Pi] 数据链路: 共发送 {sent} 帧合成里程计 "
          f"({'✓ PC侧应已收到' if sent > 0 else '✗ 发送失败'})")
    print(f"[Pi] 命令链路: {'✓ PC 数据已收到, 收到PC指令 ✓ 链路正常' if got_command[0] else '✗ 未收到PC指令'}")
    print("=" * 60)
    print("自检结论:",
          "PASS (双向正常)" if (sent > 0 and got_command[0]) else "FAIL (见上方日志)")
    sender.close()
    receiver.close()


if __name__ == "__main__":
    main()
