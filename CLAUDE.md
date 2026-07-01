# Raspberry Pi 智能小车项目文档

## 项目概述

基于树莓派的智能小车控制系统，集成 **SLAM (同步定位与建图)** 功能。
- **树莓派** 负责低延迟实时任务: 传感器驱动、里程计、扫描匹配、局部建图、运动控制
- **PC/上位机** 负责计算密集型任务: 回环检测、位姿图优化、全局地图管理

## 硬件架构

| 组件 | 说明 | 接口 |
|------|------|------|
| **树莓派** (Raspberry Pi) | 主控制器 | GPIO / UART / USB |
| **激光雷达** (LiDAR) | 环境扫描测距 | UART (`/dev/serial0`, 230400 baud) |
| **编码器** (Encoder) | 轮速/里程测量 | GPIO (A相: 17, 27; B相: 22, 10) |
| **IMU 传感器** | 姿态/加速度/角速度 | USB (`/dev/ttyUSB0`, 115200 baud) |
| **摄像头** (Camera) | 视觉采集 | USB (MJPG, 1280x720@30fps) |
| **电机驱动** (PWM) | 双路电机控制 | GPIO (PWMA:18, AIN1:23, AIN2:24; PWMB:13, BIN1:5, BIN2:6) |

## 软件架构

```
├── main.py                    # 主程序入口 (集成SLAM管线)
├── pwm.py                     # 电机PWM控制
├── udp_sender.py              # UDP通信模块 (统一帧格式)
│
├── slam/                      # SLAM模块 (运行在Pi上)
│   ├── __init__.py
│   ├── odometry.py            # 里程计 (编码器+IMU互补滤波)
│   ├── scan_matcher.py        # 相关性扫描匹配 (CSM)
│   ├── local_mapper.py        # 局部占据栅格地图构建
│   └── keyframe.py            # 关键帧选择器
│
├── PC/                        # PC端模块
│   ├── __init__.py
│   ├── slam_client.py         # PC端SLAM主控制 (UDP接收+分发)
│   ├── loop_detector.py       # 回环检测 (扫描匹配历史关键帧)
│   └── pose_graph.py          # 位姿图优化 (梯度下降)
│
├── model/
│   └── models.py              # 数据模型定义、消息类型常量、pack/unpack
│
├── robot_run/
│   ├── motion_control.py      # 运动控制 (直行+转向闭环)
│   └── udp_receiver.py        # UDP命令接收 (运动指令+SLAM反馈)
│
├── Encoder/
│   └── Encoder.py             # 编码器类 (含后台采样线程)
│
├── lidar/
│   ├── lidar_receive.py       # 雷达传感器驱动
│   ├── lidar_parser.pyx       # Cython雷达数据解析
│   └── setup.py               # Cython编译脚本
│
├── imu/
│   ├── imu.py                 # IMU传感器驱动 (含latest_yaw属性)
│   ├── imu_parser.pyx         # Cython IMU数据解析
│   └── setup_imu.py           # Cython编译脚本
│
├── camera/
│   └── test.py                # 摄像头测试脚本
│
├── joy_test/
│   ├── joystick_control.py    # 手柄控制类
│   └── test_joystick.py       # 手柄测试脚本
│
├── frp_0.66.0_linux_arm/      # frp内网穿透工具
├── SLAM_ARCHITECTURE.md       # SLAM架构详细文档
├── CLAUDE.md                  # 项目文档 (本文件)
└── README.md                  # 项目README
```

## SLAM 架构概览

```
树莓派 (低延迟实时层)
  ┌──────────┐  ┌──────────┐  ┌──────────┐
  │  LiDAR   │  │   IMU    │  │ Encoder  │
  └────┬─────┘  └────┬─────┘  └────┬─────┘
       │              │              │
       ▼              ▼              ▼
  ┌──────────────────────────┐
  │    里程计 (Odometry)      │  编码器+IMU互补滤波
  └──────────┬───────────────┘
             ▼
  ┌──────────────────────────┐
  │   扫描匹配 (ScanMatcher)  │  CSM 相关性匹配
  └──────────┬───────────────┘
             ▼
  ┌──────────────────────────┐
  │  局部建图 (LocalMapper)   │  占据栅格 + 关键帧提取
  └──────────┬───────────────┘
             │  UDP (关键帧/里程计/子图)
             ▼
PC / 上位机 (全局优化层)
  ┌──────────────────────────┐
  │   回环检测 (LoopDetector) │
  └──────────┬───────────────┘
             ▼
  ┌──────────────────────────┐
  │  位姿图优化 (PoseGraph)   │
  └──────────┬───────────────┘
             ▼  位姿修正 (UDP)
        → 树莓派里程计修正
```

详细架构见 `SLAM_ARCHITECTURE.md`

## 通信协议

### UDP 数据包格式

统一帧格式 (12字节头):
```
| 消息类型 (uint32, BE) | 时间戳 (uint64, BE, ns) | 负载长度 (uint32, BE) | 负载 (N 字节) |
```

### 消息类型定义

#### Pi → PC (传感器 + SLAM数据)

| 类型 | 值 | 说明 |
|------|----|------|
| `MSG_LIDAR` | `0x01` | 激光雷达点云 (bit-packed压缩) |
| `MSG_ENCODER` | `0x02` | 编码器增量数据 |
| `MSG_IMU` | `0x03` | IMU原始数据 (多帧打包) |
| `MSG_ODOM` | `0x04` | 里程计位姿 (28字节: x,y,θ,v,ω,ts) |
| `MSG_KEYFRAME` | `0x05` | 关键帧 (扫描+位姿) |
| `MSG_LOCAL_MAP` | `0x06` | 局部子图 (栅格数据) |

#### PC → Pi (指令 + SLAM反馈)

| 类型 | 值 | 说明 |
|------|----|------|
| `MSG_LOOP_CLOSURE` | `0x07` | 回环约束通知 (24字节) |
| `MSG_POSE_CORRECTION` | `0x08` | 全局位姿修正 (16字节) |
| `MSG_CMD_REPLY` | `0x11` | 运动控制指令 |

## 模块说明

### 1. 主程序 (`main.py`)
- 启动 `pigpio` 守护进程
- 初始化传感器、电机、SLAM组件
- 启动多线程: 雷达发送、IMU发送、SLAM处理、命令接收、运动控制
- SLAM线程: 50Hz (里程计→扫描匹配→局部建图→关键帧发送)

### 2. SLAM - 里程计 (`slam/odometry.py`)
- 互补滤波融合编码器(位移)和IMU(方向)
- 差分驱动模型: 左轮A相, 右轮B相
- 输出: 世界坐标系位姿 (x, y, θ) + 速度 (v, ω)
- 参数: wheel_radius, wheel_base, encoder_resolution, alpha

### 3. SLAM - 扫描匹配 (`slam/scan_matcher.py`)
- 相关性扫描匹配 (CSM): 暴力搜索
- 在里程计先验周围搜索窗口内寻找最佳匹配位姿
- 搜索参数可配: xy搜索范围、角度搜索范围、步长

### 4. SLAM - 局部建图 (`slam/local_mapper.py`)
- 2D占据栅格地图 (int8: -100空闲 ~ +100占据)
- Bresenham射线投射更新
- 滑动窗口: 机器人移动超阈值时自动重新居中
- 支持子图导出 (LocalMap)

### 5. SLAM - 关键帧 (`slam/keyframe.py`)
- 基于距离/角度/时间的三条件关键帧触发
- 生成 KeyFrame 数据包供UDP发送

### 6. PC - SLAM客户端 (`PC/slam_client.py`)
- UDP接收关键帧、里程计、局部子图
- 管理回环检测流程
- 发送位姿修正回树莓派

### 7. PC - 回环检测 (`PC/loop_detector.py`)
- 空间距离搜索候选 + 扫描匹配验证
- 最小关键帧间隔滤波
- 输出回环约束 (LoopClosure)

### 8. PC - 位姿图 (`PC/pose_graph.py`)
- 节点: 关键帧位姿; 边: 里程计边 + 回环边
- 简化梯度下降优化
- 第一个节点固定, 其他节点优化

### 9. 电机控制 (`pwm.py`)
- 使用 `pigpio` 硬件 PWM
- 双路电机独立控制（A/B）
- PWM 频率 1000Hz

### 10. 运动控制 (`robot_run/motion_control.py`)
- `StraightController`: 编码器闭环直行 (P控制)
- `TurnController`: IMU yaw 闭环转向
- `MotionController`: 50Hz 主控制线程

### 11. 编码器 (`Encoder/Encoder.py`)
- `pigpio` 边沿回调计数
- 后台线程 100Hz 采样
- 输出 EncoderFrame (A/B相边沿增量)

### 12. 激光雷达 (`lidar/lidar_receive.py`)
- UART 230400 baud
- Cython 加速帧解析 (`lidar_parser.pyx`)
- 自动检测完整扫描圈

### 13. IMU (`imu/imu.py`)
- 串口 115200 baud
- Cython 加速帧解析 (`imu_parser.pyx`)
- 支持 JY901 系列协议
- `latest_yaw` / `latest_gyro_z` 属性供外部实时读取

## 部署与运行

### 环境依赖

```bash
# 系统依赖
sudo apt install pigpio python3-pigpio python3-serial python3-opencv

# Python 包
pip install pigpio pyserial opencv-python numpy

# 编译 Cython 模块
cd lidar  && python3 setup.py build_ext --inplace
cd ../imu && python3 setup_imu.py build_ext --inplace
```

### 启动步骤

1. **启动 pigpio 守护进程**
   ```bash
   sudo pigpiod
   ```

2. **启动 frp 内网穿透**（如需远程访问）
   ```bash
   cd frp_0.66.0_linux_arm
   ./frpc -c frpc.toml
   ```

3. **运行树莓派主程序**
   ```bash
   python3 main.py
   ```

4. **运行PC端SLAM客户端**
   ```bash
   python3 PC/slam_client.py
   ```

### 配置参数

主要配置在 `main.py` 顶部:
```python
# 运动参数
WHEEL_RADIUS = 0.0325
WHEEL_BASE = 0.17
ENCODER_RESOLUTION = 20
ODOM_ALPHA = 0.7

# 扫描匹配
SCAN_SEARCH_XY = 0.5
SCAN_SEARCH_THETA = 15.0
SCAN_MIN_SCORE = 30.0

# 局部地图
GRID_RESOLUTION = 0.05
GRID_WIDTH = 400

# 关键帧
KF_DIST_THRESHOLD = 0.5
KF_ANGLE_THRESHOLD = 15.0
KF_TIME_THRESHOLD = 2.0
```