from dataclasses import dataclass

# 消息类型常量
MSG_LIDAR = 0x01
MSG_IMU   = 0x03


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