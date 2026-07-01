"""PC端 SLAM 客户端

接收树莓派发送的关键帧、里程计和局部子图数据，
管理回环检测、位姿图优化和全局地图构建。

通信架构 (TCP):
  - TCP Server bind 0.0.0.0:5005 : 接收 Pi 端数据 (关键帧/里程计/局部子图)
  - TCP Client connect Pi:5006    : 发送位姿修正到 Pi
"""

import socket
import struct
import threading
import time
from queue import Queue
from model.models import (
    MSG_ODOM, MSG_KEYFRAME, MSG_LOCAL_MAP,
    MSG_LOOP_CLOSURE, MSG_POSE_CORRECTION,
    OdometryFrame, KeyFrame, LocalMap, LoopClosure,
)
from PC.loop_detector import LoopDetector
from PC.pose_graph import PoseGraph


class SlamClient:
    """PC端 SLAM 主客户端

    工作流程:
    1. 接收Pi端的关键帧和里程计数据
    2. 将关键帧加入位姿图
    3. 触发回环检测
    4. 发现回环后执行位姿图优化
    5. 将位姿修正发送回Pi
    """

    def __init__(
        self,
        listen_port=5005,
        target_ip=None,         # Pi的IP地址 (用于发送修正)
        target_port=5006,       # Pi的TCP命令端口
        loop_search_radius=3.0,
        loop_min_interval=20,
        loop_match_threshold=0.6,
    ):
        # TCP 接收 (server)
        self.listen_port = listen_port
        self.target_ip = target_ip
        self.target_port = target_port

        self._server_sock = None
        self._corr_sock = None       # 发送修正的 TCP client socket
        self._corr_connected = False
        self._corr_lock = threading.Lock()
        self._running = False
        self._recv_thread = None
        self._corr_thread = None

        # SLAM 组件
        self.loop_detector = LoopDetector(
            search_radius=loop_search_radius,
            min_interval=loop_min_interval,
            match_threshold=loop_match_threshold,
        )
        self.pose_graph = PoseGraph()

        # 数据队列
        self.keyframe_queue = Queue(maxsize=100)
        self.odom_queue = Queue(maxsize=100)
        self.local_map_queue = Queue(maxsize=10)

        # 回调
        self.on_keyframe = None     # callable(KeyFrame)
        self.on_odom = None         # callable(OdometryFrame)
        self.on_local_map = None    # callable(LocalMap)
        self.on_loop = None         # callable(LoopClosure)
        self.on_correction = None   # callable(PoseCorrection)

        # 统计
        self._kf_count = 0
        self._loop_count = 0
        self._start_time = 0.0

    # =========================
    # 启动/停止
    # =========================

    def start(self):
        if self._running:
            return

        # 创建 TCP server 监听 Pi 数据
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind(("0.0.0.0", self.listen_port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(1.0)

        self._running = True
        self._start_time = time.time()

        self._recv_thread = threading.Thread(
            target=self._accept_and_recv_loop, daemon=True, name="pc-recv")
        self._recv_thread.start()

        # 后台重连 Pi:5006 用于发送修正
        if self.target_ip:
            self._corr_thread = threading.Thread(
                target=self._corr_connect_loop, daemon=True, name="pc-corr")
            self._corr_thread.start()

    def stop(self):
        self._running = False

        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
        if self._corr_thread:
            self._corr_thread.join(timeout=2.0)

        with self._corr_lock:
            if self._corr_sock:
                try:
                    self._corr_sock.close()
                except OSError:
                    pass
                self._corr_sock = None
                self._corr_connected = False

        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None

    # =========================
    # TCP Server: 接收 Pi 数据 (port 5005)
    # =========================

    def _accept_and_recv_loop(self):
        """接受 Pi 连接 → 接收帧循环 → 断连后重新 accept"""
        while self._running:
            try:
                conn, addr = self._server_sock.accept()
                print(f"[SlamClient] Pi 已连接: {addr}")
                # 进入帧接收循环
                self._recv_framed_loop(conn)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.5)

    def _recv_framed_loop(self, sock):
        """从已连接 socket 循环读取并分发消息帧"""
        sock.settimeout(0.5)
        while self._running:
            header = self._recv_exactly(sock, 12)
            if header is None:
                print("[SlamClient] Pi 断开连接")
                break

            msg_type, timestamp_ns, payload_len = struct.unpack("!IQI", header[:12])

            payload = self._recv_exactly(sock, payload_len)
            if payload is None:
                print("[SlamClient] Pi 断开连接(读payload)")
                break

            self._dispatch(msg_type, timestamp_ns, payload)

        try:
            sock.close()
        except OSError:
            pass

    @staticmethod
    def _recv_exactly(sock, n):
        """从 socket 精确读取 n 字节，失败返回 None"""
        buf = bytearray()
        while len(buf) < n:
            try:
                chunk = sock.recv(n - len(buf))
                if not chunk:
                    return None  # 连接关闭
                buf += chunk
            except socket.timeout:
                return None
            except OSError:
                return None
        return bytes(buf)

    def _dispatch(self, msg_type, timestamp_ns, payload):
        """分发收到的消息"""
        if msg_type == MSG_ODOM and len(payload) >= 28:
            try:
                odom = OdometryFrame.unpack(payload[:28])
                self.odom_queue.put(odom)
                if self.on_odom:
                    self.on_odom(odom)
            except Exception:
                pass

        elif msg_type == MSG_KEYFRAME and len(payload) >= 18:
            try:
                kf = KeyFrame.unpack(payload)
                self.keyframe_queue.put(kf)
                self._handle_keyframe(kf)
            except Exception as e:
                print(f"[SlamClient] keyframe unpack error: {e}")

        elif msg_type == MSG_LOCAL_MAP and len(payload) >= 16:
            try:
                lmap = LocalMap.unpack(payload)
                self.local_map_queue.put(lmap)
                if self.on_local_map:
                    self.on_local_map(lmap)
            except Exception as e:
                print(f"[SlamClient] local_map unpack error: {e}")

    # =========================
    # TCP Client: 发送修正到 Pi (port 5006)
    # =========================

    def _corr_connect_loop(self):
        """后台重连 Pi:5006 (指数退避)"""
        backoff = 1.0
        while self._running:
            with self._corr_lock:
                if self._corr_connected:
                    pass  # 已连接，等待
                else:
                    try:
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(5.0)
                        s.connect((self.target_ip, self.target_port))
                        s.settimeout(None)
                        self._corr_sock = s
                        self._corr_connected = True
                        backoff = 1.0
                        print(f"[SlamClient] 修正通道已连接 {self.target_ip}:{self.target_port}")
                    except OSError:
                        if self._corr_sock:
                            try:
                                self._corr_sock.close()
                            except OSError:
                                pass
                        self._corr_sock = None
            if not self._corr_connected and self._running:
                time.sleep(min(backoff, 16.0))
                backoff = min(backoff * 2, 16.0)
            else:
                time.sleep(0.5)

    def send_correction(self, correction):
        """发送位姿修正到树莓派"""
        if self.target_ip is None:
            return
        with self._corr_lock:
            if not self._corr_connected or self._corr_sock is None:
                return
            sock = self._corr_sock

        try:
            payload = correction.pack()
            header = struct.pack("!IQI", MSG_POSE_CORRECTION,
                                int(time.time() * 1e9), len(payload))
            sock.sendall(header + payload)
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            print(f"[SlamClient] send correction error: {e}")
            with self._corr_lock:
                self._corr_connected = False
                self._corr_sock = None
            try:
                sock.close()
            except OSError:
                pass

    # =========================
    # SLAM 处理
    # =========================

    def _handle_keyframe(self, kf):
        """处理新关键帧"""
        self._kf_count += 1

        # 添加到位姿图 (里程计边自动生成)
        self.pose_graph.add_keyframe(kf.id, kf.pose)

        # 回环检测
        loop = self.loop_detector.add_keyframe(kf)
        if loop is not None:
            self._loop_count += 1
            self.pose_graph.add_loop_closure(loop)
            if self.on_loop:
                self.on_loop(loop)

            # 触发位姿图优化
            corrections = self.pose_graph.optimize()
            for kf_id, corr in corrections.items():
                if self.on_correction:
                    self.on_correction(corr)
                # 发送修正到Pi
                self.send_correction(corr)

        # 用户回调
        if self.on_keyframe:
            self.on_keyframe(kf)

    # =========================
    # 查询接口
    # =========================

    def get_keyframe(self, block=True, timeout=None):
        """获取下一个关键帧"""
        return self.keyframe_queue.get(block=block, timeout=timeout)

    def get_odom(self, block=False):
        """获取最新里程计数据 (非阻塞)"""
        try:
            return self.odom_queue.get(block=block, timeout=0.01)
        except Exception:
            return None

    def get_optimized_pose(self, kf_id):
        """获取优化后的位姿"""
        return self.pose_graph.get_pose(kf_id)

    def get_all_poses(self):
        """获取所有关键帧的优化位姿"""
        return self.pose_graph.get_all_poses()

    def get_stats(self):
        """获取统计信息"""
        elapsed = time.time() - self._start_time
        return {
            "runtime_s": elapsed,
            "keyframes_received": self._kf_count,
            "loops_detected": self._loop_count,
            "graph_nodes": self.pose_graph.node_count(),
            "graph_edges": self.pose_graph.edge_count(),
            "loop_detector": self.loop_detector.get_stats(),
        }


# ==================== PC端主入口 ====================

def main():
    """PC端SLAM客户端主程序"""
    import argparse

    parser = argparse.ArgumentParser(description="PC端 SLAM 客户端")
    parser.add_argument("--port", type=int, default=5005,
                       help="监听端口 (接收Pi数据)")
    parser.add_argument("--target-ip", type=str, default=None,
                       help="树莓派IP (发送位姿修正)")
    parser.add_argument("--target-port", type=int, default=5006,
                       help="树莓派命令端口")
    parser.add_argument("--loop-radius", type=float, default=3.0,
                       help="回环检测空间搜索半径 (m)")
    parser.add_argument("--loop-interval", type=int, default=20,
                       help="最小关键帧间隔")
    parser.add_argument("--loop-threshold", type=float, default=0.6,
                       help="回环匹配置信度阈值")
    args = parser.parse_args()

    client = SlamClient(
        listen_port=args.port,
        target_ip=args.target_ip,
        target_port=args.target_port,
        loop_search_radius=args.loop_radius,
        loop_min_interval=args.loop_interval,
        loop_match_threshold=args.loop_threshold,
    )

    # 设置回调
    def on_kf(kf):
        print(f"[PC] 收到关键帧 #{kf.id}: "
              f"({kf.pose.x:.3f}, {kf.pose.y:.3f}), "
              f"点数={len(kf.points)}")

    def on_loop(loop):
        print(f"[PC] 检测到回环: {loop.kf_id_a} ↔ {loop.kf_id_b}, "
              f"置信度={loop.confidence:.2f}")

    def on_corr(corr):
        print(f"[PC] 发送位姿修正: kf={corr.kf_id}, "
              f"({corr.corrected_x:.3f}, {corr.corrected_y:.3f})")

    client.on_keyframe = on_kf
    client.on_loop = on_loop
    client.on_correction = on_corr

    client.start()
    print(f"[PC] SLAM客户端已启动 (TCP), 监听端口 {args.port}")
    print(f"[PC] 发送位姿修正到 {args.target_ip}:{args.target_port}" if args.target_ip
          else "[PC] (未设置Pi地址，不会发送位姿修正)")

    try:
        while True:
            time.sleep(10)
            stats = client.get_stats()
            print(f"[PC] 统计: 关键帧={stats['keyframes_received']}, "
                  f"回环={stats['loops_detected']}, "
                  f"图节点={stats['graph_nodes']}, 图边={stats['graph_edges']}")
    except KeyboardInterrupt:
        pass
    finally:
        print("[PC] 正在退出...")
        client.stop()
        print("[PC] 退出完成")


if __name__ == "__main__":
    main()
