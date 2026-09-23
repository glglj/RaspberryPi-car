"""雷达性能分析：测量 read / parse / pack / send 各环节耗时。"""
import time
import collections
import threading
from lidar.lidar_receive import LidarSensor
from tcp_sender import UdpSender
from model.models import MSG_LIDAR


def profile_lidar(duration=15):
    """运行雷达采集 N 秒，统计各个环节的耗时分布。"""
    lidar = LidarSensor()
    udp = UdpSender("bj.zyfrp.vip", 5005)
    stop_event = threading.Event()

    # 各环节耗时记录 (μs)
    read_times = []    # uart.read
    feed_times = []    # parser.feed
    enq_times = []     # _enqueue
    pack_times = []    # pack_frame
    send_times = []    # udp.send
    frame_times = []   # 一整帧从取出到发送完
    queue_waits = []   # get_frame 等待时间

    # ── 猴子补丁 LidarSensor ──
    orig_read_loop = lidar._read_loop

    def timed_read_loop():
        current = []
        while lidar._running:
            t0 = time.perf_counter_ns()
            data = lidar.uart.read(4096)
            t1 = time.perf_counter_ns()
            read_times.append(t1 - t0)

            if not data:
                continue

            t2 = time.perf_counter_ns()
            pkgs = lidar.parser.feed(data)
            t3 = time.perf_counter_ns()
            feed_times.append(t3 - t2)

            for pkg in pkgs:
                if pkg.get("is_start") and current:
                    t4 = time.perf_counter_ns()
                    lidar._enqueue(current)
                    t5 = time.perf_counter_ns()
                    enq_times.append(t5 - t4)
                    current = []
                for si in pkg["Si"]:
                    current.append((si["angle"], si["distance"]))

    # 停掉旧线程，等它彻底退出
    lidar.stop()
    lidar._thread.join(timeout=2)
    # 换上新函数，重启
    lidar._running = True
    lidar._read_loop = timed_read_loop
    lidar._thread = threading.Thread(target=lidar._read_loop, daemon=True)
    lidar._thread.start()

    frame_count = 0

    def lidar_loop():
        nonlocal frame_count
        while not stop_event.is_set():
            t0 = time.perf_counter_ns()
            result = lidar.get_frame(timeout=0.5)
            t1 = time.perf_counter_ns()
            queue_waits.append(t1 - t0)

            if result is None:
                continue
            ts, frame = result
            frame_count += 1

            t2 = time.perf_counter_ns()
            payload = LidarSensor.pack_frame(ts, frame)
            t3 = time.perf_counter_ns()
            pack_times.append(t3 - t2)

            t4 = time.perf_counter_ns()
            udp.send(MSG_LIDAR, ts, payload)
            t5 = time.perf_counter_ns()
            send_times.append(t5 - t4)

            frame_times.append(t5 - t0)

    t = threading.Thread(target=lidar_loop, daemon=True)
    t.start()

    print(f"采集 {duration} 秒中...")
    time.sleep(duration)
    stop_event.set()
    lidar.stop()

    # ── 统计 ──
    def stats(name, data_us):
        if not data_us:
            return f"  {name}: (无数据)"
        data_us = [d / 1000 for d in data_us]  # ns → μs
        data_us.sort()
        n = len(data_us)
        return (
            f"  {name:12s}: "
            f"avg={sum(data_us)/n:8.1f}μs  "
            f"min={data_us[0]:8.1f}μs  "
            f"max={data_us[-1]:8.1f}μs  "
            f"p50={data_us[n//2]:8.1f}μs  "
            f"count={n}"
        )

    print(f"\n{'='*65}")
    print(f"雷达性能分析结果（采样 {duration}s）")
    print(f"{'='*65}")
    print(f"  发送帧数: {frame_count}  ({frame_count/duration:.1f} Hz)")
    print()
    print("─ 后台读串口线程 ─")
    print(stats("uart.read", read_times))
    print(stats("parser.feed", feed_times))
    print(stats("_enqueue", enq_times))
    print()
    print("─ 发送线程 ─")
    print(stats("get_frame 等", queue_waits))
    print(stats("pack_frame", pack_times))
    print(stats("udp.send", send_times))
    print(stats("整帧总耗时", frame_times))

    if frame_times:
        ft = [f / 1000 for f in frame_times]
        print(f"\n  帧耗时: avg={sum(ft)/len(ft):.0f}μs  p99={sorted(ft)[int(len(ft)*0.99)]:.0f}μs")

    # CPU 估算
    total_cpu_us = 0
    for name, data in [("feed", feed_times), ("pack", pack_times)]:
        if data:
            total_cpu_us += sum(data) / 1000
    print(f"  雷达 CPU 估算: {total_cpu_us / (duration * 10000):.2f}%")


if __name__ == "__main__":
    profile_lidar(15)