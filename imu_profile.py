"""IMU 性能分析：测量 read / parse 各环节耗时。"""
import time
import collections
from imu.imu import IMUSensor


def profile_imu(duration=10):
    """运行 IMU 采集 N 秒，统计每轮 read/parse 的耗时分布。"""
    imu = IMUSensor()
    imu.start()

    read_times = []
    parse_times = []
    loop_times = []
    data_lens = []

    # 猴子补丁：给 _worker 加上计时
    orig_worker = imu._worker

    def timed_worker():
        while imu.running:
            t0 = time.perf_counter_ns()

            t1 = time.perf_counter_ns()
            imu.buffer += imu.ser.read(64)
            t2 = time.perf_counter_ns()

            imu._parse_buffer()
            t3 = time.perf_counter_ns()

            read_times.append(t2 - t1)
            parse_times.append(t3 - t2)
            loop_times.append(t3 - t0)
            data_lens.append(len(imu.buffer))

    imu._worker = timed_worker
    # 重启线程用新函数
    imu.stop()
    imu.running = True
    imu.thread = __import__('threading').Thread(target=imu._worker, daemon=True)
    imu.thread.start()

    print(f"采集 {duration} 秒中...")
    time.sleep(duration)
    imu.stop()

    if not read_times:
        print("没收到数据，检查 IMU 串口连接")
        return

    # ── 统计 ──
    def stats(name, data_us):
        data_us = [d / 1000 for d in data_us]  # ns → μs
        data_us.sort()
        n = len(data_us)
        return (
            f"  {name}: "
            f"avg={sum(data_us)/n:7.1f}μs  "
            f"min={data_us[0]:7.1f}μs  "
            f"max={data_us[-1]:7.1f}μs  "
            f"p50={data_us[n//2]:7.1f}μs  "
            f"p99={data_us[int(n*0.99)]:7.1f}μs  "
            f"count={n}"
        )

    print(f"\n{'='*60}")
    print(f"IMU 性能分析结果（采样 {duration}s，共 {len(loop_times)} 轮）")
    print(f"{'='*60}")
    print(stats("read 耗时  ", read_times))
    print(stats("parse 耗时 ", parse_times))
    print(stats("整轮耗时   ", loop_times))
    print(f"  buffer 平均长度: {sum(data_lens)/len(data_lens):.0f} 字节")
    print()

    # ── CPU 占用估算 ──
    total_us = sum(loop_times) / 1000  # 总耗时 μs
    duration_us = duration * 1_000_000
    cpu_pct = total_us / duration_us * 100
    print(f"  CPU 占用估算: {cpu_pct:.1f}%  ({total_us/1000:.1f}ms / {duration}s)")
    print(f"  平均每轮间隔: {duration*1e6/len(loop_times):.0f}μs  ({len(loop_times)/duration:.0f} Hz)")


if __name__ == "__main__":
    profile_imu(10)