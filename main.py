import threading
import pigpio
from lidar.lidar_receive import LidarSensor
from imu.imu import IMUSensor
from Encoder.Encoder import EncoderSensor
from pwm import Motor
from robot_run.motion_control import MotionController
from udp_sender import UdpSender
from robot_run.udp_receiver import UdpReceiver
from model.models import (
    MSG_LIDAR, MSG_IMU,
    CMD_STOP, CMD_STRAIGHT, CMD_TURN_LEFT, CMD_TURN_RIGHT
)


def main():
    # ---- pigpio ----
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio 未运行")
        return

    # ---- 电机 ----
    motor_a = Motor(pi, pwm_pin=18, in1=23, in2=24, freq=1000)
    motor_b = Motor(pi, pwm_pin=13, in1=5, in2=6, freq=1000)

    # ---- 传感器 ----
    lidar = LidarSensor()
    imu = IMUSensor()
    imu.start()
    encoder = EncoderSensor(pi)

    # ---- UDP ----
    udplidar = UdpSender("bj.zyfrp.vip", 5005)
    udpimu = UdpSender("am.zyfrp.vip", 5005)
    receiver = UdpReceiver(port=5006)

    # ---- 运动控制 ----
    motion = MotionController(motor_a, motor_b, encoder, imu)
    motion.start()

    stop_event = threading.Event()

    # ---- lidar 上报线程 ----
    def lidar_loop():
        while not stop_event.is_set():
            result = lidar.get_frame(timeout=0.5)
            if result is None:
                continue
            ts, frame = result
            payload = LidarSensor.pack_frame(ts, frame)
            udplidar.send(MSG_LIDAR, ts, payload)

    # ---- imu 上报线程 ----
    def imu_loop():
        while not stop_event.is_set():
            try:
                ts, payload = imu.get(timeout=0.5)
            except Exception:
                continue
            udpimu.send(MSG_IMU, ts, payload)

    # ---- 指令接收线程 ----
    def cmd_recv_loop():
        while not stop_event.is_set():
            cmd = receiver.recv(timeout=0.1)
            if cmd is None:
                continue
            if cmd.cmd_type == CMD_STOP:
                motion.stop()
            elif cmd.cmd_type == CMD_STRAIGHT:
                motion.go_straight(int(cmd.param))
            elif cmd.cmd_type == CMD_TURN_LEFT:
                motion.turn("left", cmd.param)
            elif cmd.cmd_type == CMD_TURN_RIGHT:
                motion.turn("right", cmd.param)

    # ---- 启动线程 ----
    threads = [
        threading.Thread(target=lidar_loop, daemon=True),
        threading.Thread(target=imu_loop, daemon=True),
        threading.Thread(target=cmd_recv_loop, daemon=True),
    ]
    for t in threads:
        t.start()

    print("系统运行中（lidar + imu + cmd_recv + motion_control 线程）...")
    try:
        stop_event.wait()
    except KeyboardInterrupt:
        pass
    finally:
        print("正在退出...")
        stop_event.set()
        motion.stop_controller()
        lidar.stop()
        imu.stop()
        receiver.close()
        pi.stop()
        print("退出程序")


if __name__ == "__main__":
    main()
