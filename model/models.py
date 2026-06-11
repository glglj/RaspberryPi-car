from dataclasses import dataclass

# 消息类型常量
MSG_LIDAR = 0x01
MSG_ENCODER = 0x02
MSG_IMU = 0x03
MSG_CMD = 0x10
MSG_CMD_REPLY = 0x11
MSG_CMD_ACK = 0x12

# 指令类型常量
CMD_STOP = 0x00
CMD_STRAIGHT = 0x01
CMD_TURN_LEFT = 0x02
CMD_TURN_RIGHT = 0x03


@dataclass
class MotionCommand:
    cmd_type: int      # 0=STOP, 1=STRAIGHT, 2=TURN_LEFT, 3=TURN_RIGHT
    param: float       # 速度 or 角度


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
