# cython: language_level=3
# cython: boundscheck=False
# cython: wraparound=False
# cython: cdivision=True

"""Cython 加速的相关性扫描匹配器 (Correlative Scan Matcher)

优化要点:
  1. C 级别类型化循环 —— 消除 Python 解释器开销
  2. 预计算栅格坐标偏移 —— 每次查表只需整数加法，避免浮点除法和减法
  3. 多层搜索(粗→细) —— 3倍步长粗搜 + 1倍步长精搜，候选数减少 ~80%
  4. memoryview 零拷贝访问栅格 —— 绕过 Python 的 __getitem__ 调用
  5. 降采样扫描点 —— 最多 180 个点，均匀选取
"""

import numpy as np
cimport numpy as np
from libc.math cimport cos, sin

from model.models import RobotPose


cdef class FastScanMatcher:
    """Cython 加速的扫描匹配器，API 与原 ScanMatcher 兼容"""

    cdef:
        double search_xy, search_theta, res_xy, res_theta
        int max_points, coarse_factor
    cdef public:
        double min_match_score
        double last_score
        int match_count, fail_count

    def __cinit__(
        self,
        double search_window_xy=0.5,
        double search_window_theta=15.0,
        double resolution_xy=0.05,
        double resolution_theta=1.0,
        double min_match_score=50.0,
        int max_scan_points=180,
        int coarse_factor=3,
    ):
        self.search_xy = search_window_xy
        self.search_theta = search_window_theta * 0.017453292519943295  # deg → rad
        self.res_xy = resolution_xy
        self.res_theta = resolution_theta * 0.017453292519943295
        self.min_match_score = min_match_score
        self.max_points = max_scan_points
        self.coarse_factor = coarse_factor
        self.last_score = 0.0
        self.match_count = 0
        self.fail_count = 0

    # ================================================================
    # 公开接口
    # ================================================================

    cpdef tuple match(self, list scan_points, object prior_pose, object local_map):
        """将扫描匹配到局部地图 (Cython 加速版)

        Args:
            scan_points: [(angle_rad, distance_m), ...] 激光扫描点
            prior_pose: RobotPose 里程计先验位姿
            local_map: LocalMap 局部栅格地图

        Returns:
            (RobotPose, float): 最佳匹配位姿和得分
        """
        cdef:
            int n, i, step
            double angle, dist
            list filtered

            # 地图数据
            np.int8_t[:, :] grid
            double origin_x, origin_y, inv_res
            int gw, gh

            # 扫描点 (numpy)
            np.ndarray[np.float64_t, ndim=2] points
            double[:, :] pts

            # 预分配
            np.ndarray[np.float64_t, ndim=1] rx_arr, ry_arr
            np.ndarray[np.int32_t, ndim=1] gx0_arr, gy0_arr
            double[:] rx, ry
            int[:] gx0, gy0

            # 搜索参数
            double prior_x, prior_y, prior_theta
            int x_steps, y_steps, t_steps
            double inv_res_xy

            # 循环变量
            int dxi, dyi, dti, p, gx_off, gy_off, gx, gy, valid_count
            double dx, dy, dt, cand_theta, cos_t, sin_t, score
            double best_score, best_dx, best_dy, best_dt

        # ---- 1. 过滤 & 转笛卡尔坐标 (Python 层，数据量小无所谓) ----
        filtered = []
        for angle, dist in scan_points:
            if 0.05 < dist < 20.0:
                filtered.append((dist * cos(angle), dist * sin(angle)))

        n = len(filtered)
        if n < 10:
            self.fail_count += 1
            return (prior_pose, 0.0)

        # ---- 2. 降采样 ----
        if n > self.max_points:
            step = n // self.max_points
            filtered = filtered[::step][:self.max_points]
            n = len(filtered)

        points = np.array(filtered, dtype=np.float64)
        pts = points

        # ---- 3. 提取地图数据 (零拷贝 memoryview) ----
        grid = local_map.grid
        origin_x = local_map._origin_x
        origin_y = local_map._origin_y
        inv_res = 1.0 / local_map.grid_resolution
        gw = local_map.grid_width
        gh = local_map.grid_height

        inv_res_xy = self.res_xy * inv_res  # 每个 dx/dy 步长对应多少栅格

        prior_x = prior_pose.x
        prior_y = prior_pose.y
        prior_theta = prior_pose.theta

        # ---- 4. 预分配旋转缓冲区 ----
        rx_arr = np.empty(n, dtype=np.float64)
        ry_arr = np.empty(n, dtype=np.float64)
        gx0_arr = np.empty(n, dtype=np.int32)
        gy0_arr = np.empty(n, dtype=np.int32)
        rx = rx_arr
        ry = ry_arr
        gx0 = gx0_arr
        gy0 = gy0_arr

        # ---- 5. 粗搜索 (3x 步长) ----
        x_steps = <int>(self.search_xy / (self.res_xy * self.coarse_factor))
        y_steps = x_steps
        t_steps = <int>(self.search_theta / (self.res_theta * self.coarse_factor))

        best_score = -1e300
        best_dx = best_dy = best_dt = 0.0

        _match_coarse(
            pts, n,
            grid, gw, gh, origin_x, origin_y, inv_res,
            prior_x, prior_y, prior_theta,
            self.res_xy * self.coarse_factor,
            self.res_theta * self.coarse_factor,
            x_steps, t_steps, inv_res_xy * self.coarse_factor,
            rx_arr, ry_arr, gx0_arr, gy0_arr,
            &best_score, &best_dx, &best_dy, &best_dt,
        )

        # ---- 6. 精搜索 (1x 步长, 在粗搜索结果附近) ----
        if best_score > -1e200:
            x_steps = self.coarse_factor  # ±3 个精步长
            y_steps = self.coarse_factor
            t_steps = self.coarse_factor

            _match_fine(
                pts, n,
                grid, gw, gh, origin_x, origin_y, inv_res,
                prior_x, prior_y, prior_theta,
                self.res_xy, self.res_theta,
                x_steps, t_steps, inv_res_xy,
                rx_arr, ry_arr, gx0_arr, gy0_arr,
                &best_score, &best_dx, &best_dy, &best_dt,
            )

        self.last_score = best_score

        cdef object best_pose = RobotPose(
            x=prior_x + best_dx,
            y=prior_y + best_dy,
            theta=prior_theta + best_dt,
        )

        if best_score < self.min_match_score:
            self.fail_count += 1
            return (prior_pose, best_score)

        self.match_count += 1
        return (best_pose, best_score)

    # ================================================================
    # 统计
    # ================================================================

    cpdef dict get_stats(self):
        cdef int total = self.match_count + self.fail_count
        cdef double rate = self.match_count / <double>total if total > 0 else 0.0
        return {
            "match_count": self.match_count,
            "fail_count": self.fail_count,
            "success_rate": rate,
            "last_score": self.last_score,
        }


# ================================================================
# C 级别内联搜索函数 (避免 cpdef 调用开销，持有 GIL 但不调用 Python)
# ================================================================

cdef void _match_coarse(
    double[:, :] pts,
    int n,
    np.int8_t[:, :] grid,
    int gw, int gh,
    double origin_x, double origin_y, double inv_res,
    double prior_x, double prior_y, double prior_theta,
    double step_xy, double step_theta,
    int xy_steps, int t_steps, double grid_per_step,
    double[:] rx, double[:] ry,
    int[:] gx0, int[:] gy0,
    double* best_score, double* best_dx, double* best_dy, double* best_dt,
):
    """粗搜索: step_xy = res_xy * coarse_factor, step_theta = res_theta * coarse_factor"""
    cdef:
        int dxi, dyi, dti, p, gx, gy, valid_count, gx_off_int, gy_off_int
        double dx, dy, dt, cos_t, sin_t, cand_theta, score

    for dti in range(-t_steps, t_steps + 1):
        dt = dti * step_theta
        cand_theta = prior_theta + dt
        cos_t = cos(cand_theta)
        sin_t = sin(cand_theta)

        # 旋转所有点 + 平移到世界坐标
        for p in range(n):
            rx[p] = prior_x + pts[p, 0] * cos_t - pts[p, 1] * sin_t
            ry[p] = prior_y + pts[p, 0] * sin_t + pts[p, 1] * cos_t

        # 预计算 base 栅格坐标 (dx=0, dy=0)
        for p in range(n):
            gx0[p] = <int>((rx[p] - origin_x) * inv_res)
            gy0[p] = <int>((ry[p] - origin_y) * inv_res)

        gx_off_int = <int>(grid_per_step + 0.5)  # 每步栅格偏移

        for dxi in range(-xy_steps, xy_steps + 1):
            gx_off = dxi * gx_off_int

            for dyi in range(-xy_steps, xy_steps + 1):
                gy_off = dyi * gx_off_int

                score = 0.0
                valid_count = 0

                for p in range(n):
                    gx = gx0[p] + gx_off
                    gy = gy0[p] + gy_off
                    if 0 <= gx < gw and 0 <= gy < gh:
                        score += grid[gy, gx]
                        valid_count += 1

                if valid_count >= 10 and score > best_score[0]:
                    best_score[0] = score
                    best_dx[0] = dx = dxi * step_xy
                    best_dy[0] = dy = dyi * step_xy
                    best_dt[0] = dt


cdef void _match_fine(
    double[:, :] pts,
    int n,
    np.int8_t[:, :] grid,
    int gw, int gh,
    double origin_x, double origin_y, double inv_res,
    double prior_x, double prior_y, double prior_theta,
    double step_xy, double step_theta,
    int xy_range, int t_range, double grid_per_step,
    double[:] rx, double[:] ry,
    int[:] gx0, int[:] gy0,
    double* best_score, double* best_dx, double* best_dy, double* best_dt,
):
    """精搜索: 在粗搜索结果附近以 1x 分辨率搜索"""
    cdef:
        int dxi, dyi, dti, p, gx, gy, valid_count, gx_step
        double base_dx = best_dx[0]
        double base_dy = best_dy[0]
        double base_dt = best_dt[0]
        double dx, dy, dt, cos_t, sin_t, cand_theta, score

    gx_step = <int>(grid_per_step + 0.5)

    for dti in range(-t_range, t_range + 1):
        dt = base_dt + dti * step_theta
        cand_theta = prior_theta + dt
        cos_t = cos(cand_theta)
        sin_t = sin(cand_theta)

        # 旋转所有点
        for p in range(n):
            rx[p] = prior_x + pts[p, 0] * cos_t - pts[p, 1] * sin_t
            ry[p] = prior_y + pts[p, 0] * sin_t + pts[p, 1] * cos_t

        # 预计算 base 栅格坐标
        for p in range(n):
            gx0[p] = <int>((rx[p] - origin_x) * inv_res)
            gy0[p] = <int>((ry[p] - origin_y) * inv_res)

        for dxi in range(-xy_range, xy_range + 1):
            dx = base_dx + dxi * step_xy

            for dyi in range(-xy_range, xy_range + 1):
                dy = base_dy + dyi * step_xy

                score = 0.0
                valid_count = 0

                for p in range(n):
                    gx = gx0[p] + dxi * gx_step
                    gy = gy0[p] + dyi * gx_step
                    if 0 <= gx < gw and 0 <= gy < gh:
                        score += grid[gy, gx]
                        valid_count += 1

                if valid_count >= 10 and score > best_score[0]:
                    best_score[0] = score
                    best_dx[0] = dx
                    best_dy[0] = dy
                    best_dt[0] = dt
