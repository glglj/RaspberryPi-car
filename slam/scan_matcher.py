"""相关性扫描匹配 (Correlative Scan Matching)

使用暴力搜索在里程计先验附近寻找最优位姿，
使当前激光扫描与已有局部地图的匹配得分最大化。

原理: 将激光点云投影到栅格地图上，计算每个点落在"占据"栅格上的
概率之和。遍历搜索窗口内的所有候选位姿，选择得分最高的。
"""

import math
import numpy as np
from model.models import RobotPose


class ScanMatcher:
    """相关性扫描匹配器

    在里程计先验位姿周围搜索，找到与局部地图最匹配的位姿。
    """

    def __init__(
        self,
        search_window_xy=0.5,       # 平移搜索范围 (m)
        search_window_theta=15.0,   # 旋转搜索范围 (度)
        resolution_xy=0.05,         # 平移搜索步长 (m)
        resolution_theta=1.0,       # 旋转搜索步长 (度)
        min_match_score=50.0,       # 最低匹配得分
    ):
        self.search_window_xy = search_window_xy
        self.search_window_theta = math.radians(search_window_theta)
        self.resolution_xy = resolution_xy
        self.resolution_theta = math.radians(resolution_theta)
        self.min_match_score = min_match_score

        # 统计
        self.last_score = 0.0
        self.match_count = 0
        self.fail_count = 0

    def match(self, scan_points, prior_pose, local_map):
        """将扫描匹配到局部地图

        Args:
            scan_points: [(angle_rad, distance_m), ...] 激光扫描点
            prior_pose: RobotPose 里程计先验位姿
            local_map: LocalMap 局部栅格地图

        Returns:
            (RobotPose, score): 最佳匹配位姿和得分
        """
        # 预处理扫描点: 过滤无效距离，转换到笛卡尔坐标系
        valid_points = []
        for angle, dist in scan_points:
            if dist > 0.05 and dist < 20.0:  # 距离有效范围
                px = dist * math.cos(angle)
                py = dist * math.sin(angle)
                valid_points.append((px, py))

        if len(valid_points) < 10:
            self.fail_count += 1
            return prior_pose, 0.0

        points = np.array(valid_points, dtype=np.float32)  # (N, 2)

        # 构建搜索空间
        x_steps = int(self.search_window_xy / self.resolution_xy)
        y_steps = int(self.search_window_xy / self.resolution_xy)
        theta_steps = int(self.search_window_theta / self.resolution_theta)

        best_score = -float('inf')
        best_pose = prior_pose

        # 粗搜索
        for dx_i in range(-x_steps, x_steps + 1):
            dx = dx_i * self.resolution_xy
            for dy_i in range(-y_steps, y_steps + 1):
                dy = dy_i * self.resolution_xy
                for dt_i in range(-theta_steps, theta_steps + 1):
                    dt = dt_i * self.resolution_theta

                    # 候选位姿
                    cand_x = prior_pose.x + dx
                    cand_y = prior_pose.y + dy
                    cand_theta = prior_pose.theta + dt

                    score = self._score_scan(points, cand_x, cand_y,
                                             cand_theta, local_map)
                    if score > best_score:
                        best_score = score
                        best_pose = RobotPose(x=cand_x, y=cand_y, theta=cand_theta)

        self.last_score = best_score

        if best_score < self.min_match_score:
            self.fail_count += 1
            return prior_pose, best_score

        self.match_count += 1
        return best_pose, best_score

    def _score_scan(self, points, x, y, theta, local_map):
        """计算扫描在某位姿下的地图匹配得分

        对每个扫描点: 转换到世界坐标 → 查地图占据概率 → 累加
        """
        # 旋转+平移变换
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        score = 0.0

        for px, py in points:
            # 变换到世界坐标
            wx = x + px * cos_t - py * sin_t
            wy = y + px * sin_t + py * cos_t

            # 查地图
            grid_val = local_map.get_grid_value(wx, wy)
            if grid_val is not None:
                # grid_val: -100(空闲) ~ +100(占据), 映射到得分
                # 占据点得正分，空闲点得负分
                score += grid_val

        return score

    def get_stats(self):
        """返回匹配统计"""
        total = self.match_count + self.fail_count
        rate = self.match_count / total if total > 0 else 0
        return {
            "match_count": self.match_count,
            "fail_count": self.fail_count,
            "success_rate": rate,
            "last_score": self.last_score,
        }