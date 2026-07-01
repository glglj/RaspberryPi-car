"""里程计模块: 编码器 + IMU 融合定位

使用差分驱动模型，编码器提供位移，IMU提供偏航角，
通过互补滤波融合产生平滑的位姿估计。
"""

import math
import threading
import time
from model.models import RobotPose, OdometryFrame


class Odometry:
    """差分驱动机器人里程计

    输入: 编码器增量 (A/B相边沿计数), IMU偏航角 (度)
    输出: 世界坐标系下的位姿 (x, y, theta)
    """

    def __init__(
        self,
        wheel_radius=0.0325,        # 轮子半径 (m)
        wheel_base=0.17,            # 轮距 (m)
        encoder_resolution=20,      # 编码器每圈脉冲数
        alpha=0.7,                  # IMU融合权重 (0=纯编码器, 1=纯IMU)
    ):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.encoder_resolution = encoder_resolution
        self.alpha = alpha

        # 位姿状态
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0           # 弧度, [-π, π]

        # 速度估计
        self._v = 0.0               # 线速度 (m/s)
        self._omega = 0.0           # 角速度 (rad/s)

        # 上一周期的编码器总量 (用于计算增量)
        self._last_a_total = 0
        self._last_b_total = 0
        self._last_update_ns = 0

        # 统计
        self._total_distance = 0.0  # 总行驶距离 (m)

    # =========================
    # 外部接口
    # =========================

    def update(self, a_total_edges, b_total_edges, imu_yaw_deg, timestamp_ns):
        """更新里程计

        Args:
            a_total_edges: A相编码器累计边沿数 (左轮)
            b_total_edges: B相编码器累计边沿数 (右轮)
            imu_yaw_deg: IMU当前偏航角 (度)
            timestamp_ns: 纳秒时间戳
        """
        # 计算编码器增量
        delta_a = a_total_edges - self._last_a_total
        delta_b = b_total_edges - self._last_b_total
        self._last_a_total = a_total_edges
        self._last_b_total = b_total_edges

        # 转换为轮子位移 (米)
        # 每圈: encoder_resolution 个边沿 → 2π * wheel_radius 米
        meters_per_edge = (2.0 * math.pi * self.wheel_radius) / self.encoder_resolution
        left_dist = delta_a * meters_per_edge
        right_dist = delta_b * meters_per_edge

        # 差分驱动模型
        d_center = (left_dist + right_dist) / 2.0       # 中心点位移
        d_theta_enc = (right_dist - left_dist) / self.wheel_base  # 转角 (弧度)

        # 互补滤波: IMU提供绝对方向，编码器提供位移
        # 编码器角度增量 vs IMU绝对角度 → 融合
        imu_yaw_rad = math.radians(imu_yaw_deg)

        # 使用IMU的绝对方向 + 编码器角度增量做互补
        # 如果上一时刻没有记录，直接用IMU方向初始化
        if self._last_update_ns == 0:
            self._last_yaw_imu = imu_yaw_rad
            self._theta = imu_yaw_rad  # 初始方向以IMU为准

        # IMU的推理: 新的绝对方向
        d_theta_imu = self._normalize_angle(imu_yaw_rad - self._last_yaw_imu)
        self._last_yaw_imu = imu_yaw_rad

        # 互补滤波融合
        d_theta = self.alpha * d_theta_imu + (1.0 - self.alpha) * d_theta_enc

        # 更新位姿 (使用中值法积分)
        with self._lock:
            half_theta = self._theta + d_theta / 2.0
            self._x += d_center * math.cos(half_theta)
            self._y += d_center * math.sin(half_theta)
            self._theta = self._normalize_angle(self._theta + d_theta)
            self._total_distance += abs(d_center)

            # 速度估计
            if self._last_update_ns > 0:
                dt = (timestamp_ns - self._last_update_ns) / 1e9
                if dt > 0:
                    self._v = d_center / dt
                    self._omega = d_theta / dt
            self._last_update_ns = timestamp_ns

    def update_from_encoder_frame(self, encoder_frame, imu_yaw_deg, encoder):
        """封装方法: 从 EncoderFrame + IMU yaw 更新里程计

        Args:
            encoder_frame: EncoderFrame (包含a_edges增量, b_edges增量)
            imu_yaw_deg: IMU最新偏航角
            encoder: EncoderSensor 实例 (用于获取累计值)
        """
        a_total = encoder.a_counter.total()
        b_total = encoder.b_counter.total()
        ts_ns = int(encoder_frame.timestamp * 1e9)
        self.update(a_total, b_total, imu_yaw_deg, ts_ns)

    def get_pose(self) -> RobotPose:
        """获取当前估计位姿"""
        with self._lock:
            return RobotPose(x=self._x, y=self._y, theta=self._theta)

    def get_odometry_frame(self) -> OdometryFrame:
        """获取完整的里程计帧 (用于UDP发送)"""
        with self._lock:
            return OdometryFrame(
                pose=RobotPose(x=self._x, y=self._y, theta=self._theta),
                v=self._v, omega=self._omega,
                timestamp_ns=self._last_update_ns,
            )

    def reset(self, x=0.0, y=0.0, theta=0.0):
        """重置里程计"""
        with self._lock:
            self._x = x
            self._y = y
            self._theta = theta
            self._v = 0.0
            self._omega = 0.0
            self._total_distance = 0.0
            self._last_update_ns = 0

    # =========================
    # 内部方法
    # =========================

    @staticmethod
    def _normalize_angle(theta):
        """将角度规整到 [-π, π]"""
        return ((theta + math.pi) % (2 * math.pi)) - math.pi