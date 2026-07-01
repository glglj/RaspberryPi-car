# cython: language_level=3
# cython: boundscheck=False
# cython: wraparound=False
# cython: cdivision=True

"""Cython 加速的 Bresenham 射线批量更新栅格地图"""

import numpy as np
cimport numpy as np


def bresenham_update(
    np.int8_t[:, :] grid,
    int gw, int gh,
    double origin_x, double origin_y, double inv_res,
    double rx, double ry,
    double[:, :] endpoints,
):
    """批量 Bresenham 射线更新栅格地图 (C 级别循环)

    Args:
        grid: 栅格地图 (int8, height x width)
        gw, gh: 地图宽高
        origin_x, origin_y: 地图世界坐标原点
        inv_res: 1.0 / 分辨率
        rx, ry: 机器人世界坐标
        endpoints: (N, 2) 扫描终点世界坐标
    """
    cdef:
        int n = endpoints.shape[0]
        int i
        int x0 = <int>((rx - origin_x) * inv_res)
        int y0 = <int>((ry - origin_y) * inv_res)
        int x0_save = x0, y0_save = y0
        int x1, y1
        int dx, dy, sx, sy, err, e2
        int val

    for i in range(n):
        # 重置起点到机器人位置
        x0 = x0_save
        y0 = y0_save

        x1 = <int>((endpoints[i, 0] - origin_x) * inv_res)
        y1 = <int>((endpoints[i, 1] - origin_y) * inv_res)

        dx = x1 - x0
        if dx < 0:
            dx = -dx
        dy = y1 - y0
        if dy < 0:
            dy = -dy

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            is_end = (x0 == x1) and (y0 == y1)

            if 0 <= x0 < gw and 0 <= y0 < gh:
                val = grid[y0, x0]
                if is_end:
                    if val <= 60:
                        grid[y0, x0] = val + 40
                else:
                    if val >= -80:
                        grid[y0, x0] = val - 20

            if is_end:
                break

            e2 = 2 * err
            if e2 > -dy:
                if x0 == x1:
                    break
                err -= dy
                x0 += sx
            if e2 < dx:
                if y0 == y1:
                    break
                err += dx
                y0 += sy
