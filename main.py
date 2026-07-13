"""
树莓派智能小车 - 主程序

架构: 树莓派负责低延迟SLAM (里程计 + 扫描匹配 + 局部建图)
      PC端负责回环检测和全局图优化

线程模型:
  - LidarSensor._read_loop      : UART读取+解析 (内置)
  - IMUSensor._worker           : 串口读取+解析 (内置)
  - EncoderSensor._worker       : GPIO边沿采样 (启动)
  - MotionController._worker    : 50Hz 运动控制
  - slam_thread                 : SLAM主循环 (里程计→扫描匹配→局部建图→关键帧)
  - unified_send_thread         : 1Hz 统一发送 (雷达+IMU+里程计+局部子图)
  - cmd_recv_thread             : 命令接收+SLAM反馈处理
"""

import threading
import time

import pigpio
import math

from lidar.lidar_receive import LidarSensor
from imu.imu import IMUSensor
from Encoder.Encoder import EncoderSensor
from pwm import Motor
from robot_run.motion_control import MotionController
from robot_run.udp_receiver import TcpReceiver
from tcp_sender import TcpSender
from model.models import (
    MSG_LIDAR, MSG_IMU, MSG_ODOM, MSG_KEYFRAME, MSG_LOCAL_MAP, MSG_UNIFIED,
    CMD_STOP, CMD_STRAIGHT, CMD_TURN_LEFT, CMD_TURN_RIGHT,
)
import struct

from slam.odometry import Odometry
from slam.scan_matcher import ScanMatcher
from slam.local_mapper import LocalMapper
from slam.keyframe import KeyframeSelector

# ==================== 配置参数 ====================

# 运动参数
WHEEL_RADIUS = 0.0325         # 轮子半径 (m)
WHEEL_BASE = 0.17             # 轮距 (m)
ENCODER_RESOLUTION = 20       # 编码器每圈脉冲数
ODOM_ALPHA = 0.7              # IMU融合权重

# 扫描匹配参数
SCAN_SEARCH_XY = 0.5          # 平移搜索范围 (m)
SCAN_SEARCH_THETA = 15.0      # 旋转搜索范围 (度)
SCAN_RES_XY = 0.05            # 平移搜索步长 (m)
SCAN_RES_THETA = 1.0          # 旋转搜索步长 (度)
SCAN_MIN_SCORE = 30.0         # 最低匹配得分

# 局部地图参数
GRID_RESOLUTION = 0.05        # 栅格分辨率 (m/grid)
GRID_WIDTH = 400              # 地图宽度 (grids)
GRID_HEIGHT = 400             # 地图高度 (grids)
RECENTER_THRESHOLD = 0.25     # 重新居中阈值

# 关键帧参数
KF_DIST_THRESHOLD = 0.5       # 最小位移间隔 (m)
KF_ANGLE_THRESHOLD = 15.0     # 最小旋转间隔 (度)
KF_TIME_THRESHOLD = 2.0       # 最小时间间隔 (s)

# TCP 目标
TCP_SLAM_IP = "bj.zyfrp.vip"  # 统一数据发送地址
TCP_PORT = 5001
CMD_PORT = 5006

# 统一发送
UNIFIED_SEND_INTERVAL = 1.0   # 1Hz 统一发送间隔


def main():
    # ---- pigpio ----
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio 未运行，请先执行 sudo pigpiod")
        return

    # ---- 电机 ----
    motor_a = Motor(pi, pwm_pin=18, in1=23, in2=24, freq=1000)
    motor_b = Motor(pi, pwm_pin=13, in1=5, in2=6, freq=1000)

    # ---- 传感器 ----
    lidar = LidarSensor()
    imu = IMUSensor()
    imu.start()
    encoder = EncoderSensor(pi)
    # 注意: encoder不启动后台线程
    # MotionController 和 SLAM 线程都直接使用 encoder.a_counter.total()/b_counter.total()
    # 通过 encoder.read() 读取增量 (MotionController), 通过累计值读取 (SLAM)

    # ---- TCP 通信 ----
    tcp_slam = TcpSender(TCP_SLAM_IP, TCP_PORT)
    receiver = TcpReceiver(port=CMD_PORT)

    # ---- 运动控制 ----
    motion = MotionController(motor_a, motor_b, encoder, imu)
    motion.start()

    # ---- SLAM 组件 ----
    odometry = Odometry(
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE,
        encoder_resolution=ENCODER_RESOLUTION,
        alpha=ODOM_ALPHA,
    )
    scan_matcher = ScanMatcher(
        search_window_xy=SCAN_SEARCH_XY,
        search_window_theta=SCAN_SEARCH_THETA,
        resolution_xy=SCAN_RES_XY,
        resolution_theta=SCAN_RES_THETA,
        min_match_score=SCAN_MIN_SCORE,
    )
    local_mapper = LocalMapper(
        grid_resolution=GRID_RESOLUTION,
        grid_width=GRID_WIDTH,
        grid_height=GRID_HEIGHT,
        recenter_threshold=RECENTER_THRESHOLD,
    )
    keyframe_selector = KeyframeSelector(
        dist_threshold=KF_DIST_THRESHOLD,
        angle_threshold=KF_ANGLE_THRESHOLD,
        time_threshold=KF_TIME_THRESHOLD,
    )

    stop_event = threading.Event()

    _send_errors = 0

    def _safe_send(msg_type, ts_ns, payload):
        nonlocal _send_errors
        if not tcp_slam.send(msg_type, ts_ns, payload):
            _send_errors += 1

    # ==================== 工作线程 ====================

    # ---- 统一发送线程 (1Hz) ----
    def _pack_sub(sub_type, sub_data):
        """打包子消息: | type(1B) | len(4B BE) | data |"""
        return struct.pack("!BI", sub_type, len(sub_data)) + sub_data

    def unified_send_loop():
        """1Hz 统一发送：LiDAR+IMU+Odom+Map 合并为一个 MSG_UNIFIED 包"""
        last_send_time = time.time()
        while not stop_event.is_set():
            now = time.time()
            ts_ns = int(now * 1e9)
            sub_packets = []
            total_bytes = 0

            # 1. LiDAR（非阻塞，无竞争）
            lidar_frame = lidar.get_latest_frame()
            if lidar_frame is not None:
                frame_ts, frame = lidar_frame
                packed = LidarSensor.pack_frame(frame_ts, frame)
                sub_packets.append(_pack_sub(MSG_LIDAR, packed))
                total_bytes += len(packed)

            # 2. IMU
            imu_ts, imu_payload = imu.get_latest_bundle()
            if imu_payload is not None:
                sub_packets.append(_pack_sub(MSG_IMU, imu_payload))
                total_bytes += len(imu_payload)

            # 3. 里程计
            odom_frame = odometry.get_odometry_frame()
            if odom_frame.timestamp_ns > 0:
                odom_packed = odom_frame.pack()
                sub_packets.append(_pack_sub(MSG_ODOM, odom_packed))
                total_bytes += len(odom_packed)

            # 4. 局部子图
            lmap_payload = local_mapper.get_local_map().pack()
            sub_packets.append(_pack_sub(MSG_LOCAL_MAP, lmap_payload))
            total_bytes += len(lmap_payload)

            # 组装并发送
            payload = struct.pack("B", len(sub_packets)) + b"".join(sub_packets)
            _safe_send(MSG_UNIFIED, ts_ns, payload)

            interval = now - last_send_time
            last_send_time = now
            print(f"[Unified] 已发送 {len(sub_packets)}子包, "
                  f"间隔={interval:.2f}s ({1.0/interval:.1f}Hz), "
                  f"数据={total_bytes/1024:.1f}KB")

            time.sleep(UNIFIED_SEND_INTERVAL)

    # ---- SLAM 处理线程 ----
    def slam_loop():
        """SLAM主循环: 里程计 → 扫描匹配 → 局部建图 → 关键帧发送"""
        last_scan_result = None
        last_odom_time = 0.0
        last_match_pose = None   # 上次扫描匹配时的位姿 (用于跳帧)
        loop_count = 0
        match_skip_dist = 0.05   # 移动超过 5cm 才做扫描匹配
        map_update_skip = 3      # 静止时每 N 帧才更新一次地图
        last_kf_time = time.time()  # 关键帧发送计时

        while not stop_event.is_set():
            t_start = time.time()
            loop_count += 1

            # 1. 更新里程计 (使用编码器累计值 + IMU偏航角)
            now = time.time()
            if now - last_odom_time >= 0.01:  # 最多100Hz
                yaw = imu.latest_yaw
                a_total = encoder.a_counter.total()
                b_total = encoder.b_counter.total()
                odometry.update(a_total, b_total, yaw, int(now * 1e9))
                last_odom_time = now

            # 2. 获取激光雷达扫描
            scan_result = lidar.get_frame(timeout=0.02)
            if scan_result is not None:
                last_scan_result = scan_result

            if last_scan_result is None:
                time.sleep(0.01)
                continue

            scan_ts, scan_points = last_scan_result

            # 过滤: 转换为 [(angle_rad, distance_m)] 格式 (mm → m)
            filtered_scan = [
                (math.radians(a), d / 1000.0)
                for a, d in scan_points
                if 100 < d < 20000  # 0.1m ~ 20m, 整数比较更快
            ]
            if len(filtered_scan) < 20:
                time.sleep(0.01)
                continue

            # 2.5. 跳帧: 如果机器人没怎么动，跳过本次扫描匹配
            current_pose = odometry.get_pose()
            do_match = True
            if last_match_pose is not None:
                dx = current_pose.x - last_match_pose.x
                dy = current_pose.y - last_match_pose.y
                if dx * dx + dy * dy < match_skip_dist * match_skip_dist:
                    do_match = False

            # 3. 扫描匹配: 当前扫描 vs 局部地图
            if do_match:
                t_match = time.time()
                corrected_pose, match_score = scan_matcher.match(
                    filtered_scan, current_pose, local_mapper)
                t_match = time.time() - t_match

                # 匹配成功用修正位姿，否则信任里程计
                final_pose = corrected_pose if match_score >= scan_matcher.min_match_score else current_pose
                last_match_pose = final_pose
            else:
                final_pose = current_pose
                t_match = 0

            # 4. 更新局部地图 (静止时跳过, 节省 Bresenham 开销)
            t_map = 0
            if do_match or loop_count % map_update_skip == 0:
                t_map = time.time()
                map_scan = filtered_scan[::3] if len(filtered_scan) > 120 else filtered_scan
                local_mapper.update(map_scan, final_pose)
                t_map = time.time() - t_map

            # 5. 关键帧检测和发送
            if keyframe_selector.should_create(final_pose):
                kf = keyframe_selector.create_keyframe(
                    final_pose, filtered_scan,
                    timestamp_ns=scan_ts,
                )
                _safe_send(MSG_KEYFRAME, kf.timestamp_ns, kf.pack())
                now_kf = time.time()
                interval = now_kf - last_kf_time
                last_kf_time = now_kf
                freq = 1.0 / interval if interval > 0 else 0
                print(f"[SLAM] 关键帧 #{kf.id} 已发送, "
                      f"间隔={interval:.2f}s ({freq:.1f}Hz), "
                      f"位姿=({final_pose.x:.3f}, {final_pose.y:.3f}, "
                      f"{math.degrees(final_pose.theta):.1f}°)")

            t_elapsed = time.time() - t_start

            # 定期输出状态 + 耗时 (每50轮)
            if loop_count % 50 == 0:
                g = local_mapper.grid
                occ = (g >= 30).sum()
                free = (g <= -30).sum()
                stats = scan_matcher.get_stats()
                print(f"[SLAM] 位姿=({final_pose.x:.2f},{final_pose.y:.2f}), "
                      f"栅格 占据={occ} 空闲={free}, "
                      f"匹配={stats['success_rate']:.0%} "
                      f"耗时 match={t_match*1000:.0f}ms map={t_map*1000:.0f}ms total={t_elapsed*1000:.0f}ms")

            time.sleep(0.05)  # ~20Hz, 雷达只有 5-10Hz, 不需要 50Hz

    # ---- 指令接收线程 ----
    def cmd_recv_loop():
        while not stop_event.is_set():
            result = receiver.recv(timeout=0.1)
            if result is None:
                time.sleep(0.05)  # 无连接/无数据时休眠，避免空转 CPU
                continue

            msg_type, data = result

            if msg_type == 'motion':
                if data.cmd_type == CMD_STOP:
                    motion.stop()
                elif data.cmd_type == CMD_STRAIGHT:
                    motion.go_straight(int(data.param))
                elif data.cmd_type == CMD_TURN_LEFT:
                    motion.turn("left", data.param)
                elif data.cmd_type == CMD_TURN_RIGHT:
                    motion.turn("right", data.param)

            elif msg_type == 'pose_correction':
                # PC端发来的全局位姿修正
                print(f"[SLAM] 接收到位姿修正: kf={data.kf_id}, "
                      f"({data.corrected_x:.3f}, {data.corrected_y:.3f}, "
                      f"{math.degrees(data.corrected_theta):.1f}°)")
                # 更新里程计位姿
                odometry.reset(
                    x=data.corrected_x,
                    y=data.corrected_y,
                    theta=data.corrected_theta,
                )

            elif msg_type == 'loop_closure':
                print(f"[SLAM] 检测到回环: {data.kf_id_a} ↔ {data.kf_id_b}, "
                      f"置信度={data.confidence:.2f}")

    # ---- 启动所有线程 ----
    threads = [
        threading.Thread(target=slam_loop, daemon=True, name="slam"),
        threading.Thread(target=unified_send_loop, daemon=True, name="unified"),
        threading.Thread(target=cmd_recv_loop, daemon=True, name="cmd"),

    ]
    for t in threads:
        t.start()

    print("=" * 50)
    print("系统运行中:")
    print("  [slam]    SLAM处理 (里程计+扫描匹配+局部建图+关键帧)")
    print("  [unified] 1Hz 统一发送 (雷达+IMU+里程计+局部子图)")
    print("  [cmd]     指令接收 + SLAM反馈处理")

    print("  [motion]  运动控制 (50Hz)")
    print("=" * 50)

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
        encoder.stop()
        receiver.close()
        tcp_slam.close()
        pi.stop()

        # 打印统计
        stats = scan_matcher.get_stats()
        print(f"扫描匹配: 成功{stats['match_count']}次 / "
              f"失败{stats['fail_count']}次 "
              f"(成功率{stats['success_rate']:.1%})")
        print(f"TCP发送失败: {_send_errors} 次")
        print("退出程序")


if __name__ == "__main__":
    main()