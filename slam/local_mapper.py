"""局部栅格地图构建器

维护以机器人当前位置为中心的局部2D占据栅格地图。
使用Bresenham射线投射更新栅格概率。
当地图窗口需要移动时，转出旧区域作为子图。
"""

import math
import numpy as np
import time
from model.models import LocalMap


class LocalMapper:
    """局部栅格地图

    维护一个滑动窗口的占据栅格地图，机器人始终在地图中心附近。
    当机器人移动超过阈值时，地图重新居中。
    """

    def __init__(
        self,
        grid_resolution=0.05,       # 栅格分辨率 (m/grid)
        grid_width=400,             # 地图宽度 (grids)
        grid_height=400,            # 地图高度 (grids)
        recenter_threshold=0.25,    # 重新居中阈值 (占窗口比例)
        hit_prob=0.75,              # 击中概率增量
        miss_prob=0.35,             # 未击中概率减量
        occ_thresh=0.6,             # 占据阈值
        free_thresh=0.4,            # 空闲阈值
    ):
        self.grid_resolution = grid_resolution
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.recenter_threshold = recenter_threshold
        self.hit_prob = hit_prob
        self.miss_prob = miss_prob
        self.occ_thresh = occ_thresh
        self.free_thresh = free_thresh

        # 概率栅格: log-odds 表示 (值越大越可能占据)
        # 使用int8存储, 映射: -100 = 空闲(概率0), 0 = 未知(概率0.5), +100 = 占据(概率1)
        self.grid = np.zeros((grid_height, grid_width), dtype=np.int8)

        # 地图在世界坐标系中的原点 (左下角)
        self._origin_x = 0.0
        self._origin_y = 0.0

        # 机器人位置 (用于判断是否需要重新居中)
        self._robot_x = 0.0
        self._robot_y = 0.0

        # 更新的栅格数量
        self._update_count = 0

    # =========================
    # 地图更新
    # =========================

    def update(self, scan_points, robot_pose):
        """用一次激光扫描更新地图

        Args:
            scan_points: [(angle_rad, distance_m), ...] 激光点 (机器人坐标系)
            robot_pose: RobotPose 机器人位姿 (世界坐标系)
        """
        self._robot_x = robot_pose.x
        self._robot_y = robot_pose.y

        # 检查是否需要重新居中
        self._maybe_recenter(robot_pose.x, robot_pose.y)

        # 机器人位置 (栅格坐标)
        rx, ry = self._world_to_grid(robot_pose.x, robot_pose.y)
        if not self._in_bounds(rx, ry):
            return

        # 对每个扫描点执行 Bresenham 射线更新
        for angle, dist in scan_points:
            if dist <= 0.02 or dist >= 20.0:
                continue

            # 扫描终点在世界坐标系中的位置
            world_angle = robot_pose.theta + angle
            end_wx = robot_pose.x + dist * math.cos(world_angle)
            end_wy = robot_pose.y + dist * math.sin(world_angle)
            ex, ey = self._world_to_grid(end_wx, end_wy)

            # Bresenham 射线: 从机器人到终点
            self._bresenham_update(rx, ry, ex, ey)

        self._update_count += 1

    def _bresenham_update(self, x0, y0, x1, y1):
        """Bresenham直线绘制: 射线路径上的栅格标记为空闲, 终点标记为占据"""
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            # 更新当前栅格: 路径点 = 空闲, 终点 = 占据
            is_end = (x0 == x1 and y0 == y1)
            if self._in_bounds(x0, y0):
                if is_end:
                    self._update_grid_cell(x0, y0, hit=True)
                else:
                    self._update_grid_cell(x0, y0, hit=False)

            if is_end:
                break

            e2 = 2 * err
            if e2 >= dy:
                if x0 == x1:
                    break
                err += dy
                x0 += sx
            if e2 <= dx:
                if y0 == y1:
                    break
                err += dx
                y0 += sy

    def _update_grid_cell(self, gx, gy, hit):
        """更新单个栅格的概率 (log-odds简化)"""
        current = self.grid[gy, gx]
        if hit:
            # 增加占据概率: 向 +100 方向移动
            self.grid[gy, gx] = min(100, current + 40)
        else:
            # 减少占据概率: 向 -100 方向移动
            self.grid[gy, gx] = max(-100, current - 20)

    # =========================
    # 坐标转换
    # =========================

    def _world_to_grid(self, wx, wy):
        """世界坐标 → 栅格坐标"""
        gx = int((wx - self._origin_x) / self.grid_resolution)
        gy = int((wy - self._origin_y) / self.grid_resolution)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        """栅格坐标 → 世界坐标 (栅格中心)"""
        wx = self._origin_x + (gx + 0.5) * self.grid_resolution
        wy = self._origin_y + (gy + 0.5) * self.grid_resolution
        return wx, wy

    def _in_bounds(self, gx, gy):
        """检查栅格坐标是否在地图范围内"""
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

    # =========================
    # 地图滑动窗口
    # =========================

    def _maybe_recenter(self, wx, wy):
        """当机器人靠近地图边缘时重新居中"""
        if self._origin_x == 0.0 and self._origin_y == 0.0:
            # 首次初始化: 将机器人放在地图中心
            self._origin_x = wx - (self.grid_width / 2) * self.grid_resolution
            self._origin_y = wy - (self.grid_height / 2) * self.grid_resolution
            return

        half_w = self.grid_width * self.grid_resolution / 2
        half_h = self.grid_height * self.grid_resolution / 2
        center_x = self._origin_x + half_w
        center_y = self._origin_y + half_h

        offset_x = wx - center_x
        offset_y = wy - center_y

        threshold_x = half_w * self.recenter_threshold
        threshold_y = half_h * self.recenter_threshold

        if abs(offset_x) > threshold_x or abs(offset_y) > threshold_y:
            self._recenter(wx, wy)

    def _recenter(self, wx, wy):
        """重新居中地图"""
        new_origin_x = wx - (self.grid_width / 2) * self.grid_resolution
        new_origin_y = wy - (self.grid_height / 2) * self.grid_resolution

        # 计算新旧原点之间的偏移 (栅格单位)
        dx_grid = int((new_origin_x - self._origin_x) / self.grid_resolution)
        dy_grid = int((new_origin_y - self._origin_y) / self.grid_resolution)

        # 平移地图
        new_grid = np.zeros_like(self.grid)
        if abs(dx_grid) < self.grid_width and abs(dy_grid) < self.grid_height:
            # 计算重叠区域
            src_x1 = max(0, dx_grid)
            src_y1 = max(0, dy_grid)
            src_x2 = min(self.grid_width, self.grid_width + dx_grid)
            src_y2 = min(self.grid_height, self.grid_height + dy_grid)

            dst_x1 = max(0, -dx_grid)
            dst_y1 = max(0, -dy_grid)
            dst_x2 = min(self.grid_width, self.grid_width - dx_grid)
            dst_y2 = min(self.grid_height, self.grid_height - dy_grid)

            src_w = src_x2 - src_x1
            src_h = src_y2 - src_y1
            dst_w = dst_x2 - dst_x1
            dst_h = dst_y2 - dst_y1

            copy_w = min(src_w, dst_w)
            copy_h = min(src_h, dst_h)

            if copy_w > 0 and copy_h > 0:
                new_grid[dst_y1:dst_y1 + copy_h, dst_x1:dst_x1 + copy_w] = \
                    self.grid[src_y1:src_y1 + copy_h, src_x1:src_x1 + copy_w]

        self.grid = new_grid
        self._origin_x = new_origin_x
        self._origin_y = new_origin_y

    # =========================
    # 查询接口
    # =========================

    def get_grid_value(self, wx, wy):
        """查询世界坐标点的栅格占据值 (-100 ~ 100, None表示出界)"""
        gx, gy = self._world_to_grid(wx, wy)
        if self._in_bounds(gx, gy):
            return int(self.grid[gy, gx])
        return None

    def get_local_map(self) -> LocalMap:
        """导出当前局部地图"""
        return LocalMap(
            origin_x=self._origin_x,
            origin_y=self._origin_y,
            width=self.grid_width,
            height=self.grid_height,
            resolution=self.grid_resolution,
            data=self.grid.copy(),
        )

    def get_occupancy_grid(self):
        """返回二值占据栅格 (0=空闲/未知, 1=占据) 用于扫描匹配"""
        occupied = self.grid >= 30  # 阈值转为二值
        return occupied.astype(np.int8)

    def get_robot_grid_pos(self):
        """返回机器人在栅格中的坐标"""
        return self._world_to_grid(self._robot_x, self._robot_y)

    def save_debug_image(self, filepath, robot_pose=None):
        """保存地图为 PPM 图片

        颜色编码:
          黑色 = 占据 (grid >= 30)
          白色 = 空闲 (grid <= -30)
          灰色 = 未知 (其他)
          红色十字 = 机器人位置
        """
        h, w = self.grid.shape
        rgb = np.zeros((h, w, 3), dtype=np.uint8)

        occupied = self.grid >= 30
        free = self.grid <= -30
        unknown = ~occupied & ~free

        rgb[occupied] = [0, 0, 0]
        rgb[free] = [255, 255, 255]
        rgb[unknown] = [128, 128, 128]

        if robot_pose is not None:
            rx, ry = self._world_to_grid(robot_pose.x, robot_pose.y)
            if self._in_bounds(rx, ry):
                rgb[max(0, ry-2):min(h, ry+3), rx, :] = [255, 0, 0]
                rgb[ry, max(0, rx-2):min(w, rx+3), :] = [255, 0, 0]

        with open(filepath, 'wb') as f:
            f.write(f"P6\n{w} {h}\n255\n".encode())
            f.write(rgb.tobytes())

    def reset(self):
        """重置地图"""
        self.grid.fill(0)
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._update_count = 0