import threading
import queue
from lidar.lidar_receive import LidarSensor
from imu.imu import IMUSensor
from udp_sender import UdpSender
from model.models import MSG_LIDAR, MSG_IMU


def main():
    lidar = LidarSensor()
    imu = IMUSensor()
    imu.start()

    udp = UdpSender("bj.zyfrp.vip", 5005)
    stop_event = threading.Event()

    def lidar_loop():
        while not stop_event.is_set():
            result = lidar.get_frame(timeout=0.5)
            if result is None:
                continue
            ts, frame = result
            payload = LidarSensor.pack_frame(ts, frame)
            # DEBUG: 打印最后3个点的原始角度
            tail_angles = [f"{p[0]:.1f}" for p in frame[-3:]]

            udp.send(MSG_LIDAR, ts, payload)

    def imu_loop():
        while not stop_event.is_set():
            try:
                ts, item = imu.get(timeout=0.5)
            except queue.Empty:
                continue
            payload = IMUSensor.pack(item)
            udp.send(MSG_IMU, ts, payload)

    t_lidar = threading.Thread(target=lidar_loop, daemon=True)
    t_imu = threading.Thread(target=imu_loop, daemon=True)
    t_lidar.start()
    t_imu.start()

    print("系统运行中（双线程模式）...")
    try:
        stop_event.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        lidar.stop()
        imu.stop()
        print("退出程序")


if __name__ == "__main__":
    main()