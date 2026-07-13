import struct
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

# ==================== 消息类型常量 ====================

# Pi → PC: 原始传感器数据
MSG_LIDAR = 0x01
MSG_ENCODER = 0x02
MSG_IMU = 0x03

# Pi → PC: SLAM数据
MSG_ODOM = 0x04
MSG_KEYFRAME = 0x05
MSG_LOCAL_MAP = 0x06

# Pi → PC: 统一数据包 (LiDAR + IMU + Odometry + LocalMap 合并发送)
MSG_UNIFIED = 0x09

# PC → Pi: SLAM反馈
MSG_LOOP_CLOSURE = 0x07
MSG_POSE_CORRECTION = 0x08

# 运动控制指令
MSG_CMD = 0x10
MSG_CMD_REPLY = 0x11
MSG_CMD_ACK = 0x12

# ==================== 指令类型常量 ====================
CMD_STOP = 0x00
CMD_STRAIGHT = 0x01
CMD_TURN_LEFT = 0x02
CMD_TURN_RIGHT = 0x03


# ==================== 数据模型 ====================

@dataclass
class MotionCommand:
    """运动控制指令 (PC → Pi)"""
    cmd_type: int      # 0=STOP, 1=STRAIGHT, 2=TURN_LEFT, 3=TURN_RIGHT
    param: float       # 速度 or 角度


@dataclass
class RobotPose:
    """机器人位姿 (2D)"""
    x: float
    y: float
    theta: float       # 弧度, 范围 [-π, π]

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    @staticmethod
    def from_tuple(t: Tuple[float, float, float]) -> 'RobotPose':
        return RobotPose(x=t[0], y=t[1], theta=t[2])


@dataclass
class OdometryFrame:
    """里程计输出帧"""
    pose: RobotPose         # 世界坐标系位姿
    v: float                # 线速度 (m/s)
    omega: float            # 角速度 (rad/s)
    timestamp_ns: int       # 纳秒时间戳

    def pack(self) -> bytes:
        """打包为 MSG_ODOM 格式:
        | x(f) | y(f) | theta(f) | v(f) | omega(f) | ts(Q) | = 28 bytes"""
        return struct.pack("<fffffQ",
            self.pose.x, self.pose.y, self.pose.theta,
            self.v, self.omega, self.timestamp_ns)

    @staticmethod
    def unpack(data: bytes) -> 'OdometryFrame':
        x, y, theta, v, omega, ts = struct.unpack("<fffffQ", data)
        return OdometryFrame(
            pose=RobotPose(x=x, y=y, theta=theta),
            v=v, omega=omega, timestamp_ns=ts)


@dataclass
class KeyFrame:
    """SLAM关键帧"""
    id: int
    pose: RobotPose                     # 全局位姿
    points: List[Tuple[float, float]]   # [(angle_rad, distance_m), ...]
    timestamp_ns: int

    def pack(self) -> bytes:
        """打包为 MSG_KEYFRAME 格式:
        | id(I) | x(f) | y(f) | theta(f) | N(H) | N*(angle(f), dist(f)) | ts(Q) |"""
        header = struct.pack("<IfffH",
            self.id, self.pose.x, self.pose.y, self.pose.theta,
            len(self.points))
        body = b''.join(
            struct.pack("<ff", a, d) for a, d in self.points
        )
        tail = struct.pack("<Q", self.timestamp_ns)
        return header + body + tail

    @staticmethod
    def unpack(data: bytes) -> 'KeyFrame':
        header_len = 4 + 4 + 4 + 4 + 2  # I f f f H = 18
        kf_id, x, y, theta, n = struct.unpack("<IfffH", data[:header_len])
        points = []
        offset = header_len
        for _ in range(n):
            a, d = struct.unpack("<ff", data[offset:offset + 8])
            points.append((a, d))
            offset += 8
        ts = struct.unpack("<Q", data[offset:offset + 8])[0]
        return KeyFrame(
            id=kf_id, pose=RobotPose(x=x, y=y, theta=theta),
            points=points, timestamp_ns=ts)


@dataclass
class LocalMap:
    """局部栅格子图"""
    origin_x: float             # 地图原点X (世界坐标, 地图左下角)
    origin_y: float             # 地图原点Y (世界坐标)
    width: int                  # 宽度 (grids)
    height: int                 # 高度 (grids)
    resolution: float           # 分辨率 (m/grid)
    data: np.ndarray            # 地图数据 (height x width, int8)

    def pack(self) -> bytes:
        """打包为 MSG_LOCAL_MAP 格式:
        | ox(f) | oy(f) | w(H) | h(H) | res(f) | data(w*h bytes) |"""
        header = struct.pack("<ffHHf",
            self.origin_x, self.origin_y,
            self.width, self.height, self.resolution)
        map_bytes = self.data.astype(np.int8).tobytes()
        return header + map_bytes

    def get_grid_value(self, wx, wy):
        """查询世界坐标点的栅格占据值 (-100 ~ 100, None表示出界)"""
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return int(self.data[gy, gx])
        return None

    @staticmethod
    def unpack(data: bytes) -> 'LocalMap':
        header_len = 4 + 4 + 2 + 2 + 4  # f f H H f = 16
        ox, oy, w, h, res = struct.unpack("<ffHHf", data[:header_len])
        map_data = np.frombuffer(
            data[header_len:header_len + w * h], dtype=np.int8
        ).reshape((h, w))
        return LocalMap(origin_x=ox, origin_y=oy,
                        width=w, height=h, resolution=res, data=map_data)


@dataclass
class LoopClosure:
    """回环约束 (PC → Pi)"""
    kf_id_a: int
    kf_id_b: int
    dx: float
    dy: float
    dtheta: float
    confidence: float

    def pack(self) -> bytes:
        """打包为 MSG_LOOP_CLOSURE 格式:
        | a(I) | b(I) | dx(f) | dy(f) | dtheta(f) | conf(f) | = 24 bytes"""
        return struct.pack("<IIffff",
            self.kf_id_a, self.kf_id_b,
            self.dx, self.dy, self.dtheta, self.confidence)

    @staticmethod
    def unpack(data: bytes) -> 'LoopClosure':
        a, b, dx, dy, dt, conf = struct.unpack("<IIffff", data)
        return LoopClosure(kf_id_a=a, kf_id_b=b,
                           dx=dx, dy=dy, dtheta=dt, confidence=conf)


@dataclass
class PoseCorrection:
    """全局位姿修正 (PC → Pi)"""
    kf_id: int
    corrected_x: float
    corrected_y: float
    corrected_theta: float

    def pack(self) -> bytes:
        """打包为 MSG_POSE_CORRECTION 格式:
        | kf_id(I) | x(f) | y(f) | theta(f) | = 16 bytes"""
        return struct.pack("<Ifff",
            self.kf_id, self.corrected_x,
            self.corrected_y, self.corrected_theta)

    @staticmethod
    def unpack(data: bytes) -> 'PoseCorrection':
        kf_id, x, y, theta = struct.unpack("<Ifff", data)
        return PoseCorrection(kf_id=kf_id,
            corrected_x=x, corrected_y=y, corrected_theta=theta)


# ==================== IMU 数据模型 (保持兼容) ====================

@dataclass
class IMUAccel:
    ax: float
    ay: float
    az: float
    temp: float


@dataclass
class IMUGyro:
    gx: float
    gy: float
    gz: float


@dataclass
class IMUAngle:
    roll: float
    pitch: float
    yaw: float


@dataclass
class IMUMag:
    mx: float
    my: float
    mz: float


@dataclass
class IMUQuat:
    q0: float
    q1: float
    q2: float
    q3: float