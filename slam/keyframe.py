"""关键帧选择和数据结构

根据距离、角度和时间条件决定何时生成新的关键帧。
关键帧包含: 激光扫描 + 位姿 + 时间戳，发送给PC用于全局SLAM。
"""

import math
import time
from model.models import RobotPose, KeyFrame


class KeyframeSelector:
    """关键帧选择器

    当机器人移动超过阈值时触发新关键帧。
    """

    def __init__(
        self,
        dist_threshold=0.5,        # 最小位移间隔 (m)
        angle_threshold=15.0,      # 最小旋转间隔 (度)
        time_threshold=2.0,        # 最小时间间隔 (s)
    ):
        self.dist_threshold = dist_threshold
        self.angle_threshold = math.radians(angle_threshold)
        self.time_threshold = time_threshold

        self._last_keyframe_pose = None
        self._last_keyframe_time = 0.0
        self._kf_id = 0

    def should_create(self, current_pose, current_time=None):
        """判断是否应该创建新关键帧

        Args:
            current_pose: RobotPose 当前位姿
            current_time: float 当前时间 (s), None则用time.time()

        Returns:
            bool: 是否需要新关键帧
        """
        if current_time is None:
            current_time = time.time()

        if self._last_keyframe_pose is None:
            return True

        # 时间条件
        dt = current_time - self._last_keyframe_time
        if dt >= self.time_threshold:
            return True

        # 距离条件
        dx = current_pose.x - self._last_keyframe_pose.x
        dy = current_pose.y - self._last_keyframe_pose.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist >= self.dist_threshold:
            return True

        # 角度条件
        dtheta = abs(self._normalize_angle(
            current_pose.theta - self._last_keyframe_pose.theta))
        if dtheta >= self.angle_threshold:
            return True

        return False

    def create_keyframe(self, current_pose, scan_points, timestamp_ns=None):
        """创建一个新关键帧

        Args:
            current_pose: RobotPose 当前位姿
            scan_points: [(angle_rad, distance_m), ...] 激光扫描
            timestamp_ns: int 纳秒时间戳

        Returns:
            KeyFrame 新关键帧
        """
        self._kf_id += 1
        self._last_keyframe_pose = RobotPose(
            x=current_pose.x, y=current_pose.y, theta=current_pose.theta)
        self._last_keyframe_time = time.time()

        if timestamp_ns is None:
            timestamp_ns = int(time.time() * 1e9)

        return KeyFrame(
            id=self._kf_id,
            pose=RobotPose(x=current_pose.x, y=current_pose.y,
                          theta=current_pose.theta),
            points=list(scan_points),
            timestamp_ns=timestamp_ns,
        )

    def reset(self):
        """重置选择器状态"""
        self._last_keyframe_pose = None
        self._last_keyframe_time = 0.0
        self._kf_id = 0

    @staticmethod
    def _normalize_angle(theta):
        return ((theta + math.pi) % (2 * math.pi)) - math.pi