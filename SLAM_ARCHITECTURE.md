# SLAM 架构文档

## 架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi (低延迟实时层)                │
│                                                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │  LiDAR   │  │   IMU    │  │ Encoder  │  │  Motor   │   │
│  │ 驱动     │  │ 驱动     │  │ 驱动     │  │ 控制     │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
│       │              │              │              │         │
│       ▼              ▼              ▼              │         │
│  ┌──────────────────────────┐                    │         │
│  │    里程计 (Odometry)      │◄───────────────────┘         │
│  │  编码器 + IMU 融合定位    │                              │
│  └──────────┬───────────────┘                              │
│             │ 初始位姿                                      │
│             ▼                                               │
│  ┌──────────────────────────┐                              │
│  │   扫描匹配 (ScanMatcher)  │  ← 当前扫描 vs 局部地图       │
│  │   CSM (相关性扫描匹配)    │                              │
│  └──────────┬───────────────┘                              │
│             │ 修正后位姿                                    │
│             ▼                                               │
│  ┌──────────────────────────┐                              │
│  │  局部建图 (LocalMapper)   │                              │
│  │  局部栅格地图 + 关键帧提取│                              │
│  └──────────┬───────────────┘                              │
│             │ 关键帧 / 局部子图                              │
│             ▼                                               │
│  ┌──────────────────────────┐                              │
│  │    UDP 发送              │                              │
│  └──────────────────────────┘                              │
└─────────────────────────┬───────────────────────────────────┘
                          │  UDP
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    PC / 上位机 (全局优化层)                   │
│                                                             │
│  ┌──────────────────────────┐                              │
│  │    UDP 接收              │                              │
│  └──────────┬───────────────┘                              │
│             │                                               │
│             ▼                                               │
│  ┌──────────────────────────┐                              │
│  │   回环检测 (LoopDetector) │  ← 扫描匹配历史关键帧         │
│  └──────────┬───────────────┘                              │
│             │ 回环约束                                      │
│             ▼                                               │
│  ┌──────────────────────────┐                              │
│  │  位姿图优化 (PoseGraph)   │  ← 全局一致性优化             │
│  └──────────┬───────────────┘                              │
│             │ 全局地图                                      │
│             ▼                                               │
│  ┌──────────────────────────┐                              │
│  │   全局地图管理 / 可视化   │                              │
│  └──────────────────────────┘                              │
└─────────────────────────────────────────────────────────────┘
```

## 职责划分

| 层 | 位置 | 模块 | 延迟要求 | 计算量 |
|----|------|------|----------|--------|
| **传感器驱动** | Pi | LiDAR/IMU/Encoder 驱动 | 实时 | 低 |
| **运动控制** | Pi | PWM + 闭环控制 | 实时(50Hz) | 低 |
| **里程计** | Pi | Odometry (编码器+IMU融合) | 高(100Hz) | 低 |
| **扫描匹配** | Pi | ScanMatcher (CSM) | 中(10-20Hz) | 中 |
| **局部建图** | Pi | LocalMapper (局部栅格地图) | 中(5-10Hz) | 中 |
| **关键帧提取** | Pi | KeyframeSelector | 低(1-5Hz) | 低 |
| **回环检测** | PC | LoopDetector | 低(按需) | 高 |
| **位姿图优化** | PC | PoseGraph | 低(按需) | 高 |
| **全局地图** | PC | GlobalMapper | 低 | 中 |
| **可视化** | PC | MapViewer | 低 | 低 |

## 模块设计

### 1. 里程计模块 (`slam/odometry.py`)

**职责**: 融合编码器和IMU数据，提供高频位姿估计。

**输入**:
- 编码器增量 (A/B相脉冲差, 来自 `EncoderSensor`)
- IMU偏航角 (yaw, 来自 `IMUSensor`)

**输出**:
- 位姿 `(x, y, θ)` — 世界坐标系下的位置和朝向
- 线速度 `v`, 角速度 `ω`

**算法**:
```
左轮位移 = (ΔA_left * 轮子周长) / 编码器分辨率
右轮位移 = (ΔA_right * 轮子周长) / 编码器分辨率
位移_d = (左轮位移 + 右轮位移) / 2
转向_dθ_enc = (右轮位移 - 左轮位移) / 轮距

// 互补滤波融合 IMU yaw
dθ = α * dθ_enc + (1-α) * dθ_imu

x += d * cos(θ + dθ/2)
y += d * sin(θ + dθ/2)
θ += dθ
```

**参数**:
- `wheel_radius`: 轮子半径 (m)
- `wheel_base`: 轮距 (m)
- `encoder_resolution`: 编码器每圈脉冲数
- `alpha`: IMU/编码器融合权重 (0-1)

### 2. 扫描匹配模块 (`slam/scan_matcher.py`)

**职责**: 使用相关性扫描匹配(CSM)将当前激光扫描与已有的局部地图对齐，修正里程计累积误差。

**输入**:
- 当前激光扫描 (距离数组)
- 里程计提供的先验位姿
- 局部栅格地图 (来自 `LocalMapper`)

**输出**:
- 修正后的位姿 `(x, y, θ)`
- 匹配置信度

**算法**:
```
1. 以里程计位姿为中心，构建搜索窗口 (x±Δx, y±Δy, θ±Δθ)
2. 对每个候选位姿，计算扫描点在地图中的占据概率之和
3. 选择得分最高的候选位姿
4. 如果最高得分 > 阈值，返回修正后的位姿
5. 否则，信任里程计位姿
```

**参数**:
- `search_window_xy`: 平移搜索范围 (默认 ±0.5m)
- `search_window_theta`: 旋转搜索范围 (默认 ±15°)
- `resolution_xy`: 平移搜索步长 (默认 0.05m)
- `resolution_theta`: 旋转搜索步长 (默认 1°)
- `min_score`: 最低接受得分

### 3. 局部建图模块 (`slam/local_mapper.py`)

**职责**: 维护以机器人当前位置为中心的局部栅格地图，负责地图更新、子图生成和关键帧提取。

**输入**:
- 修正后的激光扫描 + 位姿
- 栅格地图参数

**输出**:
- 局部栅格地图 (2D numpy array)
- 子图 (submap): 当机器人移动超出窗口时，转存的旧区域
- 关键帧: 筛选后发送给PC的扫描+位姿对

**栅格地图**:
```
- 分辨率: 0.05m/grid (可配)
- 大小: 400x400 grids = 20m x 20m (可配)
- 数据类型: int8 (0=未知, -100=空闲, +100=占据)
- 更新: Bresenham 射线投射
- 中心跟随: 机器人移动超过窗口1/4时重新居中
```

**关键帧选取策略**:
1. 距离条件: 距上一关键帧 > `keyframe_dist_threshold` (默认 0.5m)
2. 角度条件: 旋转超过 `keyframe_angle_threshold` (默认 15°)
3. 时间条件: 距上一关键帧 > `keyframe_time_threshold` (默认 2s)
4. 满足任一条件则发送关键帧

### 4. PC端: 回环检测 (`PC/loop_detector.py`)

**职责**: 检测当前关键帧与历史关键帧之间的回环闭合。

**输入**:
- 最新关键帧 (扫描 + 位姿)
- 历史关键帧数据库

**输出**:
- 回环约束 `(id_i, id_j, relative_pose, confidence)`

**算法**:
```
1. 对最新关键帧，在历史关键帧中搜索空间距离近但不时间相邻的候选
2. 对每个候选，执行扫描匹配 (CSM)
3. 如果匹配得分 > 阈值，记录回环约束
4. 验证回环一致性 (可选: 几何一致性检查)
```

### 5. PC端: 位姿图优化 (`PC/pose_graph.py`)

**职责**: 构建并优化位姿图，消除累积误差。

**节点**: 每个关键帧是一个节点，包含估计位姿
**边**:
- 里程计边: 相邻关键帧之间的相对位姿
- 回环边: 回环检测发现的非相邻关键帧之间的约束

**优化**: 使用非线性最小二乘 (LM/Gauss-Newton) 优化位姿图
**简化实现**: 使用信息滤波或在线梯度下降

### 6. PC端: SLAM客户端 (`PC/slam_client.py`)

**职责**: PC端主控程序，接收Pi端数据，管理全局SLAM流程。

**功能**:
- UDP接收线程: 接收关键帧、局部子图、里程计数据
- 维护全局关键帧数据库
- 触发回环检测
- 触发位姿图优化
- 重建全局地图
- 提供可视化接口

## 通信协议

### UDP 消息类型扩展

| 类型 | 值 | 方向 | 说明 |
|------|----|------|------|
| `MSG_LIDAR` | `0x01` | Pi → PC | 激光雷达原始扫描（保留，用于实时显示） |
| `MSG_ENCODER` | `0x02` | Pi → PC | 编码器原始数据 |
| `MSG_IMU` | `0x03` | Pi → PC | IMU原始数据 |
| **`MSG_ODOM`** | **`0x04`** | **Pi → PC** | **里程计位姿数据（新增）** |
| **`MSG_KEYFRAME`** | **`0x05`** | **Pi → PC** | **关键帧（扫描+位姿）（新增）** |
| **`MSG_LOCAL_MAP`** | **`0x06`** | **Pi → PC** | **局部子图（新增）** |
| **`MSG_LOOP_CLOSURE`** | **`0x07`** | **PC → Pi** | **回环约束通知（新增）** |
| **`MSG_POSE_CORRECTION`** | **`0x08`** | **PC → Pi** | **全局位姿修正（新增）** |
| `MSG_CMD_REPLY` | `0x11` | PC → Pi | 运动控制指令（已有） |

### 新消息负载格式

**MSG_ODOM (0x04)**
```
| x (float32 LE) | y (float32 LE) | theta (float32 LE) |
| v (float32 LE) | omega (float32 LE) |
| timestamp (uint64 LE) |
共 28 字节
```

**MSG_KEYFRAME (0x05)**
```
| kf_id (uint32 LE) | x (float32 LE) | y (float32 LE) | theta (float32 LE) |
| num_points (uint16 LE) |
| [angle1 (float32 LE) | dist1 (float32 LE) | ... angleN | distN] |
| timestamp (uint64 LE) |
首部 18 字节 + N*8 字节扫描点
```

**MSG_LOCAL_MAP (0x06)**
```
| origin_x (float32 LE) | origin_y (float32 LE) |
| width (uint16 LE) | height (uint16 LE) |
| resolution (float32 LE) |
| map_data (width*height 字节, int8, 行优先) |
首部 12 字节 + width*height 字节
```

**MSG_LOOP_CLOSURE (0x07, PC→Pi)**
```
| kf_id_a (uint32 LE) | kf_id_b (uint32 LE) |
| dx (float32 LE) | dy (float32 LE) | dtheta (float32 LE) |
| confidence (float32 LE) |
共 24 字节
```

**MSG_POSE_CORRECTION (0x08, PC→Pi)**
```
| kf_id (uint32 LE) |
| corrected_x (float32 LE) | corrected_y (float32 LE) | corrected_theta (float32 LE) |
共 16 字节
```

## 数据流

```
┌────────────┐
│  编码器    │──────┐
│  (100Hz)   │      │
└────────────┘      ▼
              ┌──────────────┐
┌────────────┐│  里程计      │──→ 位姿(x,y,θ) @ 100Hz
│   IMU      ││  (Odometry)  │
│  (100Hz)   ││              │
└────────────┘└──────┬───────┘
                     │ 先验位姿
                     ▼
┌────────────┐┌──────────────┐
│  激光雷达  ││  扫描匹配    │──→ 修正位姿 @ 10Hz
│  (10Hz)    ││  (CSM)       │
└────────────┘└──────┬───────┘
                     │ 修正位姿 + 扫描
                     ▼
              ┌──────────────┐
              │  局部建图    │──→ 局部栅格地图
              │  (LocalMap)  │
              └──────┬───────┘
                     │
          ┌──────────┼──────────┐
          ▼          ▼          ▼
    关键帧(UDP)  子图(UDP)   里程计(UDP)
          │          │          │
          └──────────┼──────────┘
                     ▼
              ┌──────────────┐
              │  PC SLAM     │
              │  客户端      │
              └──────┬───────┘
                     │
          ┌──────────┼──────────┐
          ▼          ▼          ▼
     回环检测   位姿图优化   全局地图
```

## 文件结构

```
D:\pythonfile\RaspberryPi-car\
├── slam/                           # SLAM模块 (运行在Pi上)
│   ├── __init__.py
│   ├── odometry.py                 # 里程计 (编码器+IMU融合)
│   ├── scan_matcher.py             # 相关性扫描匹配 (CSM)
│   ├── local_mapper.py             # 局部栅格地图 + 关键帧
│   └── keyframe.py                 # 关键帧数据结构和选择器
│
├── PC/                             # PC端模块
│   ├── __init__.py
│   ├── slam_client.py              # PC端SLAM主控制
│   ├── loop_detector.py            # 回环检测
│   ├── pose_graph.py               # 位姿图优化
│   └── map_viewer.py               # 可视化 (预留)
│
├── model/
│   └── models.py                   # [修改] 新增消息类型和数据结构
│
├── main.py                         # [修改] 集成SLAM模块
├── tcp_sender.py                   # [修改] 支持新消息类型
├── robot_run/
│   ├── udp_receiver.py             # [修改] 接收回环/位姿修正
│   └── motion_control.py           # [修改] 修复latest_yaw问题
│
├── lidar/
│   ├── lidar_receive.py            # (保持不变)
│   └── lidar_parser.pyx            # (保持不变)
│
├── imu/
│   ├── imu.py                      # [修改] 添加latest_yaw属性
│   └── imu_parser.pyx              # (保持不变)
│
├── Encoder/
│   └── Encoder.py                  # (保持不变)
│
├── pwm.py                          # (保持不变)
├── camera/                         # (保持不变)
└── joy_test/                       # (保持不变)
```

## 配置参数

```python
# SLAM 配置 (可放到 slam/config.py 或 main.py 顶部)

SLAM_CONFIG = {
    # 里程计
    "wheel_radius": 0.0325,        # 轮子半径 (m)
    "wheel_base": 0.17,            # 轮距 (m)
    "encoder_resolution": 20,      # 每圈脉冲数 (电机霍尔编码器)
    "alpha": 0.7,                  # IMU融合权重 (越大越信任IMU)

    # 扫描匹配
    "search_window_xy": 0.5,       # 平移搜索范围 (m)
    "search_window_theta": 15.0,   # 旋转搜索范围 (度)
    "resolution_xy": 0.05,         # 平移搜索步长 (m)
    "resolution_theta": 1.0,       # 旋转搜索步长 (度)
    "min_match_score": 50.0,       # 最低匹配得分

    # 局部地图
    "grid_resolution": 0.05,       # 栅格分辨率 (m/grid)
    "grid_width": 400,             # 地图宽度 (grid)
    "grid_height": 400,            # 地图高度 (grid)
    "recenter_threshold": 0.25,    # 重新居中阈值 (相对窗口大小)

    # 关键帧
    "keyframe_dist": 0.5,          # 最小位移间隔 (m)
    "keyframe_angle": 15.0,        # 最小旋转间隔 (度)
    "keyframe_time": 2.0,          # 最小时间间隔 (s)

    # 回环检测 (PC端)
    "loop_search_radius": 3.0,     # 搜索半径 (m)
    "loop_min_interval": 20,       # 最小关键帧间隔
    "loop_match_threshold": 0.75,  # 回环匹配置信度阈值
}
```

## 启动流程

### Pi端 (`main.py`)

```python
1. 启动 pigpio 守护进程
2. 初始化传感器 (LiDAR, IMU, Encoder)
3. 初始化里程计 (Odometry)
4. 初始化扫描匹配 (ScanMatcher)
5. 初始化局部建图 (LocalMapper)
6. 初始化电机控制 (PWM + MotionController)
7. 初始化 UDP 通信 (发送 + 接收)
8. 启动各工作线程:
   - 传感器采集线程 (已有)
   - SLAM 处理线程 (新增): 里程计→扫描匹配→局部建图→关键帧发送
   - 运动控制线程 (已有)
   - UDP 发送线程 (已有, 扩展)
   - 命令接收线程 (已有)
9. 等待停止信号
```

### PC端 (`PC/slam_client.py`)

```python
1. 初始化 UDP 接收 (绑定 5005 端口)
2. 初始化回环检测器
3. 初始化位姿图
4. 启动工作线程:
   - UDP 接收线程: 解析消息, 分发到对应处理器
   - SLAM 处理线程: 回环检测 → 位姿图优化 → 发送修正
   - 可视化线程 (可选)
5. 等待停止信号
```

## 线程模型

```
Pi 端线程:
├── LidarSensor._read_loop       (Daemon, UART读取+解析)
├── IMUSensor._worker            (Daemon, 串口读取+解析)
├── EncoderSensor._worker        (Daemon, GPIO边沿采样)
├── MotionController._worker     (Daemon, 50Hz 运动控制)
├── slam_thread                  (Daemon, SLAM主循环)
├── lidar_send_thread            (Daemon, 雷达UDP发送)
├── imu_send_thread              (Daemon, IMU UDP发送)
└── cmd_recv_thread              (Daemon, 命令接收)

PC 端线程:
├── recv_thread                  (Daemon, UDP接收+解析)
└── slam_thread                  (Daemon, 回环检测+优化)
```