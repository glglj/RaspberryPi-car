# Raspberry Pi 智能小车项目文档

## 项目概述

基于树莓派的智能小车控制系统，集成多传感器数据采集与 UDP 远程通信功能。小车通过雷达、编码器、IMU 和摄像头感知环境，通过 UDP 协议与上位机通信，支持远程指令控制。

## 硬件架构

| 组件 | 说明 | 接口 |
|------|------|------|
| **树莓派** (Raspberry Pi) | 主控制器 | GPIO / UART / USB |
| **激光雷达** (LiDAR) | 环境扫描测距 | UART (`/dev/serial0`, 230400 baud) |
| **编码器** (Encoder) | 轮速/里程测量 | GPIO (A相: 17, 27; B相: 22, 10) |
| **IMU 传感器** | 姿态/加速度/角速度 | USB (`/dev/ttyUSB0`, 9600 baud) |
| **摄像头** (Camera) | 视觉采集 | USB (MJPG, 1280x720@30fps) |
| **电机驱动** (PWM) | 双路电机控制 | GPIO (PWMA:18, AIN1:23, AIN2:24; PWMB:13, BIN1:5, BIN2:6) |

## 软件架构

```
├── main.py                    # 主程序入口
├── pwm.py                     # 电机PWM控制
├── udp_sender.py              # UDP通信模块
├── Encoder_to_PC.py           # 编码器简易读取（旧版）
├── Encoder/
│   └── Encoder.py             # 编码器类（含后台采样线程）
├── lidar/
│   ├── lidar_to_PC.py         # 雷达传感器驱动
│   ├── lidar_parser.pyx       # 雷达数据解析（Cython加速）
│   ├── lidar_parser.c         # Cython生成的C代码
│   └── lidar_parser.cpython-311-aarch64-linux-gnu.so  # 编译后的so库
├── imu/
│   └── imu.py                 # IMU传感器驱动（含后台采样线程）
├── camera/
│   └── test.py                # 摄像头测试脚本
├── model/
│   └── models.py              # 数据模型定义（dataclass）
├── frp_0.66.0_linux_arm/      # frp内网穿透工具
│   ├── frpc                   # frp客户端
│   ├── frps                   # frp服务端
│   ├── frpc.ini / frpc.toml   # 客户端配置
│   └── frps.toml              # 服务端配置
└── CLAUDE.md                  # AI辅助开发说明
```

## 通信协议

### UDP 数据包格式

所有 UDP 数据包采用统一帧格式：

```
| 魔数 (2B) | 消息类型 (2B) | 负载长度 (4B) | 负载 (N B) |
|  0xAA55   |   uint16      |    uint32     |   payload  |
```

### 消息类型定义

| 类型 | 值 | 方向 | 说明 |
|------|----|------|------|
| `MSG_LIDAR` | `0x01` | 小车 → 上位机 | 激光雷达点云数据 |
| `MSG_ENCODER` | `0x02` | 小车 → 上位机 | 编码器增量数据 |
| `MSG_CMD_PULL` | `0x10` | 小车 → 上位机 | 拉取远程指令 |
| `MSG_CMD_REPLY` | `0x11` | 上位机 → 小车 | 远程指令下发 |
| `MSG_CMD_ACK` | `0x12` | 小车 → 上位机 | 指令执行确认 |

### 负载格式

**雷达数据 (MSG_LIDAR)**
```
| 角度1 (float, 4B) | 距离1 (float, 4B) | 角度2 (float, 4B) | 距离2 (float, 4B) | ... |
```

**编码器数据 (MSG_ENCODER)**
```
| A相增量 (uint32, 4B) | B相增量 (uint32, 4B) |
```

## 模块说明

### 1. 主程序 (`main.py`)
- 启动 `pigpio` 守护进程
- 初始化各传感器和 UDP 通信
- 启动 UDP 发送线程（20Hz 循环）
- 接收并处理远程指令

### 2. 电机控制 (`pwm.py`)
- 使用 `pigpio` 硬件 PWM
- 双路电机独立控制（A/B）
- PWM 频率 1000Hz

### 3. UDP 通信 (`udp_sender.py`)
- 基于 UDP 的二进制协议通信
- 支持多传感器数据打包发送
- 内置指令拉取与响应机制

### 4. 编码器 (`Encoder/Encoder.py`)
- 使用 `pigpio` 回调检测边沿
- 后台线程定时采样（可配置频率）
- 支持 A/B 双相编码器

### 5. 激光雷达 (`lidar/lidar_to_PC.py`)
- 通过 UART 读取雷达数据
- Cython 加速解析（`lidar_parser.pyx`）
- 自动检测完整扫描圈（角度跨度 ≥ 358°）

### 6. IMU (`imu/imu.py`)
- 串口读取 IMU 数据（9600 baud）
- 后台线程持续采集
- 支持数据模型：加速度、角速度、角度、磁场、四元数

### 7. 摄像头 (`camera/test.py`)
- MJPG 格式，1280x720@30fps
- 支持曝光、增益、白平衡控制

## 部署与运行

### 环境依赖

```bash
# 系统依赖
sudo apt install pigpio python3-pigpio python3-serial python3-opencv

# Python 包
pip install pigpio pyserial opencv-python
```

### 启动步骤

1. **启动 frp 内网穿透**（如需远程访问）
   ```bash
   cd frp_0.66.0_linux_arm
   ./frpc -c frpc.toml
   ```

2. **运行主程序**
   ```bash
   python3 main.py