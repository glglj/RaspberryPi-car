"""相关性扫描匹配 (Correlative Scan Matching)

优先使用 Cython 加速版本 (scan_matcher_fast)，编译失败时回退到纯 Python 版本。
"""

try:
    from slam.scan_matcher_fast import FastScanMatcher as ScanMatcher
except ImportError:
    # ---- 纯 Python 回退版本 ----
    import math
    import numpy as np
    from model.models import RobotPose

    class ScanMatcher:
        """相关性扫描匹配器 (纯 Python 回退版)"""

        def __init__(
            self,
            search_window_xy=0.5,
            search_window_theta=15.0,
            resolution_xy=0.05,
            resolution_theta=1.0,
            min_match_score=50.0,
        ):
            self.search_window_xy = search_window_xy
            self.search_window_theta = math.radians(search_window_theta)
            self.resolution_xy = resolution_xy
            self.resolution_theta = math.radians(resolution_theta)
            self.min_match_score = min_match_score

            self.last_score = 0.0
            self.match_count = 0
            self.fail_count = 0

        def match(self, scan_points, prior_pose, local_map):
            valid_points = []
            for angle, dist in scan_points:
                if 0.05 < dist < 20.0:
                    px = dist * math.cos(angle)
                    py = dist * math.sin(angle)
                    valid_points.append((px, py))

            if len(valid_points) < 10:
                self.fail_count += 1
                return prior_pose, 0.0

            points = np.array(valid_points, dtype=np.float32)

            x_steps = int(self.search_window_xy / self.resolution_xy)
            y_steps = int(self.search_window_xy / self.resolution_xy)
            theta_steps = int(self.search_window_theta / self.resolution_theta)

            best_score = -float('inf')
            best_pose = prior_pose

            for dx_i in range(-x_steps, x_steps + 1):
                dx = dx_i * self.resolution_xy
                for dy_i in range(-y_steps, y_steps + 1):
                    dy = dy_i * self.resolution_xy
                    for dt_i in range(-theta_steps, theta_steps + 1):
                        dt = dt_i * self.resolution_theta
                        cand_x = prior_pose.x + dx
                        cand_y = prior_pose.y + dy
                        cand_theta = prior_pose.theta + dt
                        score = self._score_scan(points, cand_x, cand_y,
                                                 cand_theta, local_map)
                        if score > best_score:
                            best_score = score
                            best_pose = RobotPose(x=cand_x, y=cand_y,
                                                  theta=cand_theta)

            self.last_score = best_score
            if best_score < self.min_match_score:
                self.fail_count += 1
                return prior_pose, best_score
            self.match_count += 1
            return best_pose, best_score

        def _score_scan(self, points, x, y, theta, local_map):
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            score = 0.0
            for px, py in points:
                wx = x + px * cos_t - py * sin_t
                wy = y + px * sin_t + py * cos_t
                grid_val = local_map.get_grid_value(wx, wy)
                if grid_val is not None:
                    score += grid_val
            return score

        def get_stats(self):
            total = self.match_count + self.fail_count
            rate = self.match_count / total if total > 0 else 0
            return {
                "match_count": self.match_count,
                "fail_count": self.fail_count,
                "success_rate": rate,
                "last_score": self.last_score,
            }
