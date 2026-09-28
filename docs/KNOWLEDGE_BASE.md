# 项目知识库（系统级权威版）

> **维护约定**：本文件是整个端云协同系统的单一权威知识库，供人类审查与后续 AI 开发使用。
> 协议、架构、端口、模块职责的任何变更，须**同步更新本文件**并在 §8 决策日志追加记录。
> PC 侧另有一份精简版 `RaspberryPi-car-pc/docs/KNOWLEDGE_BASE.md`（只含 PC 侧专属信息，协议与架构以本文件为准）。
>
> 版本：2026-09-28 v1.0 ｜ 基于第一阶段全库只读分析 + 作者答疑（14 问全部回复）
> 标记约定：【事实】=代码/实测可证；【推断】=有证据的推断；【待验证】=作者确认未标定/未测试；【决策】=作者明确拍板。

---

## 1. 项目定位与当前阶段

- 课题：《基于端云协同架构的巡检机器人 SLAM 与目标识别系统设计与实现》（北科大，开题报告 v3 为纲，位于 `D:\桌面\毕设\`）。
- **已完成**：端侧实时 SLAM 管线（里程计/扫描匹配/局部建图/关键帧）、端云 TCP 通信链路、回环检测+位姿图优化（含仿真验证）、H264 视频回传、frp 双向穿透。
- **未开始**：目标检测（3.2.4）、语义地图+GLM 智能体（3.2.6）、A* 规划、对比实验（3.2.5）、数据记录回放工具。
- 实验场景基线【决策】：20m 闭环回路仿真 ≈ 计划实验场景；**无夜间巡检需求**。

## 2. 技术栈与环境（实测）

| 层 | 内容 | 版本/证据 |
|---|---|---|
| 端侧硬件 | 树莓派 aarch64 (Debian 12, 内核 6.12.25+rpt-rpi-v8)；单线雷达(UART `/dev/serial0` 230400, **10Hz**, 有圈标志, 已调好)；IMU(USB `/dev/ttyUSB0` 115200, JY901 系列协议 0x51-0x59)；编码器(GPIO 17/27/22/10)；L298N 双电机；USB 摄像头(**仅 MJPG**, 1280x720@30fps) | SSH 实测 + v4l2-ctl + 作者确认 |
| 端侧软件 | Python 3.11.2 / numpy 1.24.2 / pyserial 3.5 / pigpio 1.78 / Cython 0.29.32 / ffmpeg 5.1.6；**无 opencv/matplotlib/PIL 需求**（PIL 仅地图保存函数用，该线程已注释） | Pi 侧 pip3 list 实测 |
| 云侧软件 | Python 3.11.2 venv / numpy 2.2.6 / opencv 4.12.0 / matplotlib 3.10.8 / pillow 12.1.0；ffmpeg 在 `D:\ffmpeg` | PC venv pip list |
| 通信 | frp 0.66.0（Pi `/opt/frp`）+ frpmgr（PC `D:/frp`）；全 TCP 自定义二进制协议 | 联调实测 |

## 3. 仓库与部署拓扑

```
D:\桌面\毕设\code\
├── RaspberryPi-car\       端侧仓库（权威源）→ 部署到 Pi ~/Desktop/raspberry_car（含完整 git 历史）
│   └─ GitHub remote: glglj/RaspberryPi-car.git
├── RaspberryPi-car-pc\    云侧仓库（权威源），PC 端 .venv 运行；无 remote（本地 git + tar 备份）
├── frp配置.txt             Pi 端 frpc_zyfrp 配置源（→ /opt/frp/frpc_zyfrp.toml）
└── 本机已映射端口\          PC frpmgr 导出的映射配置
```

### 端口分配（定稿【决策】43.227.71.58 为长期方案）

| 链路 | 公网端点 | 落地 | 用途 |
|---|---|---|---|
| Pi→PC 数据 | `43.227.71.58:44310` (TCP) | PC `:8080` | SLAM 统一数据流 |
| Pi→PC 视频 | `43.227.71.58:42011` (TCP) | PC `:8888` | H264 视频流 |
| PC→Pi 命令 | `bj.zyfrp.vip:8002` (TCP) | Pi `:8002` | 运动指令/位姿修正 |
| 预留 | `bj.zyfrp.vip:8001`(TCP), 8003/8004(UDP) | Pi 同端口 | 备用（UDP 不再有切换计划，见 D1） |
| SSH | `39.102.115.36:45704` | Pi `:22` | 公钥登录，`frpc.service` |

- `bj.zyfrp.vip:5001/5005`（旧数据通道）**弃用不回头**【决策】。
- 树莓派关机时 SSH 会拒绝连接，属正常（frp 隧道不通）。

## 4. 模块地图（端侧 `RaspberryPi-car` / 云侧 `RaspberryPi-car-pc`）

### 端侧
| 模块 | 职责 | 关键位置 |
|---|---|---|
| `main.py` | 组装+线程编排+全部配置中心 | `main(dry_run)` :110；线程列表 :403 |
| `slam/odometry.py` | 编码器位移+IMU 航向互补滤波，中值积分 | :94-101 |
| `slam/scan_matcher.py` | CSM；try 导入 Cython（`scan_matcher_fast`），失败回退纯 Python | :6-13 |
| `slam/local_mapper.py` | 400×400 int8 log-odds 简化栅格 + Bresenham + 滑窗重居中 | :85, 218-277 |
| `slam/keyframe.py` | 关键帧触发：时间/距离/角度**任一满足即触发（OR）** | :49-66 |
| `slam/*_fast.pyx` | Cython 加速（粗→细两级搜索 / memoryview 射线更新） | `setup_scan.py` |
| `robot_run/motion_control.py` | 50Hz：直行 P 控(kp=0.5) + IMU 闭环转向(2°阈值) | :13, 36-41 |
| `robot_run/udp_receiver.py` | **TCP** 命令服务端（文件名遗留 udp，类名 TcpReceiver） | :33 |
| `tcp_sender.py` | TCP 客户端，指数退避重连 | :22-31 |
| `Encoder/Encoder.py` | pigpio EITHER_EDGE 计数 + 100Hz 采样线程 | :19-22 |
| `lidar/lidar_receive.py` | 雷达驱动+整圈组装+20bit 位压缩（9bit 角度/0.9° + 11bit 距离/10mm） | :83-100 |
| `imu/imu.py + imu_parser.pyx` | JY901 解析、`latest_yaw`/`latest_gyro_z`、周期 bundle 打包 | :14-31 |
| `camera/h264_streamer.py` | H264 推流，`--encode native|libx264` 双模式 | :53-72 |
| `pwm.py` | L298N 单路电机（PWM 1000Hz，duty 0-255） | :5-27 |
| `joy_test/` | 手柄控制（`/dev/input/js0`，Start 键安全锁）——**仅调试用，不集成不维护**【决策 D7】 | — |
| `tools/link_check/` | 双向链路自检（合成帧，无需传感器） | — |
| `tools/`（本仓库） | — | — |

### 云侧
| 模块 | 职责 | 关键位置 |
|---|---|---|
| `PC/slam_client.py` | 主客户端：收数据→分发→回环→优化→回传修正 | `_handle_keyframe` :345-372 |
| `PC/loop_detector.py` | 候选筛选(半径3m+间隔20KF) + **似然场**匹配 + 两级搜索 | `_build_field`, `_scan_match` |
| `PC/pose_graph.py` | **SE(2) 边** + 高斯牛顿(LM λ=1e-4, Huber δ=0.8)，首节点固定 | `optimize()` |
| `PC/slam_viewer.py` | OpenCV 实时可视化（CJK 字体处理） | :18-46 |
| `PC/video_receiver.py` | H264 接收：ffplay 实时 / `--save` 录制双模式 | — |
| `PC/map_viewer.py` | **过时**（硬编码 5001），勿用 | :10 |
| `control/car_controller.py` | **遗留死代码**（UDP 5 字节裸协议发 5006）——按作者要求**标记保留不修**【决策 D7】 | — |
| `model/models.py` | 协议数据模型（与端侧**逐字节相同**，diff 已验证） | — |
| `tools/sim_loopclosure/` | 回环+位姿图仿真验证器（`--seed 7` 效果好） | — |
| `tools/link_check/` | 链路自检 PC 侧 | — |

## 5. 系统架构与数据流

### Pi 端线程模型（`main.py`）
```
pigpiod 守护进程（前置条件）
├─ LidarSensor._read_loop    UART→Cython解析→整圈入队(maxsize=1, 丢旧)
├─ IMUSensor._worker         串口→Cython解析→周期bundle→latest_yaw/gyro_z
├─ MotionController._worker  50Hz: STOP/STRAIGHT/TURN
├─ slam_loop (~20Hz)         里程计(≤100Hz)→取圈→过滤(0.1-20m,≥20点)→
│                            静止跳帧(≤5cm)→CSM→建图(静止每3帧)→关键帧→TCP事件发送
├─ unified_send_loop (1Hz)   MSG_UNIFIED: 雷达圈+IMU+里程计+局部子图(≈157KB/帧)
├─ cmd_recv_loop             0.1s轮询: 运动指令 / 位姿修正(odometry.reset 直跳) / 回环(仅打印)
└─ (map_save_loop 已注释)
```

### 端云闭环
```
Pi ──MSG_KEYFRAME(事件) + MSG_UNIFIED(1Hz)──TCP→ 43.227.71.58:44310 ──frp→ PC:8080
    slam_client._handle_keyframe:
        pose_graph.add_keyframe（里程计边自动生成）
        loop_detector.add_keyframe → [回环] → pose_graph.optimize()
        → 逐帧 send_correction ──TCP→ bj.zyfrp.vip:8002 ──frp→ Pi:8002
    cmd_recv_loop → odometry.reset(x,y,θ)   ← 直接跳变，无平滑（见 §10 R9）
```

**语义要点**：
- Pi 收到 `MSG_LOOP_CLOSURE` 只打印不消费；约束融合只发生在 PC 位姿图，Pi 只接受最终修正值。
- `slam_viewer._global_map` 只是命名，实际存最新局部子图；**全局地图融合未实现**（作者确认无隐藏设计）。
- 位姿修正**平滑回注未实现**（作者确认无既定方案，见 §13 路线图）。

## 6. 通信协议规范（权威，变更须同步本节）

### 6.1 帧封装（传输层，**大端**）
```
| msg_type !I (4B) | timestamp_ns !Q (8B) | payload_len !I (4B) | payload (N) |
```
- 封装：`tcp_sender.py:84`；解析：`udp_receiver.py:93`、`slam_client.py`。
- **帧头恒为 16 字节**（历史上两端各出过一次"12字节"bug，见 §9）。

### 6.2 消息类型与载荷（`model/models.py:9-34`，载荷**小端**）
| 值 | 名称 | 载荷格式 | 大小 | 方向 |
|---|---|---|---|---|
| 0x01 | MSG_LIDAR | ts(`<Q`) + N×3B 位压缩 | 8+3N | Pi→PC |
| 0x03 | MSG_IMU | count(1B) + 帧(type 1B + floats) | 可变 | Pi→PC |
| 0x04 | MSG_ODOM | `<fffffQ` x,y,θ,v,ω,ts | 28B | Pi→PC |
| 0x05 | MSG_KEYFRAME | `<IfffH` + N×8B(角度,距离) + `<Q` | 18+8N+8 | Pi→PC |
| 0x06 | MSG_LOCAL_MAP | `<ffHHf` + w×h int8 | 16+160000 | Pi→PC |
| 0x09 | MSG_UNIFIED | count(1B) + 子包(1B type + **`!I`大端**长度 + data) | 可变 | Pi→PC |
| 0x07 | MSG_LOOP_CLOSURE | `<IIffff` a,b,dx,dy,dθ,conf | 24B | PC→Pi |
| 0x08 | MSG_POSE_CORRECTION | `<Ifff` kf_id,x,y,θ | 16B | PC→Pi |
| 0x11 | MSG_CMD_REPLY | `<Bf` cmd,param | 5B | PC→Pi |

**端序混用警示**：帧头大端、UNIFIED 子包长度大端、payload 数据小端——同一字节流内两种端序并存，解析代码必须分别指定。

### 6.3 空间约定（2026-09-23 定稿）
- 里程计边与回环边统一为 **SE(2) 体坐标系相对位姿** `T_a⁻¹·T_b`（`pose_graph._relative_pose`；`loop_detector._scan_match` 输出即此约定）。
- θ 单位弧度 [-π,π]；IMU yaw 单位为**度**（`odometry.update` 内转换）。
- 运动指令 `<Bf`：0=STOP / 1=STRAIGHT(param=速度-100~100) / 2=TURN_LEFT / 3=TURN_RIGHT(param=角度°)。
- 栅格值 int8：-100 空闲 ~ +100 占据，0 未知；占据判定阈 ≥30；hit+40/miss-20。

## 7. 关键参数与标定状态

| 参数 | 值 | 状态 |
|---|---|---|
| 轮半径 / 轮距 | 0.0325m / 0.17m | 【待验证】初始设置，未做过直线+旋转标定 |
| 编码器分辨率 | 20 | 【待验证】物理含义（边沿/圈? 含减速比?）未确认；初始设置未动，硬件未换 |
| IMU 融合权重 α | 0.7 | 【待验证】依据未记录；且 IMU yaw 本身计划弃用（D6），α 将随重构失效 |
| CSM 搜索窗 | ±0.5m / ±15°，步长 0.05m/1°，最低分 30 | 已用（实车出图验证） |
| 栅格 | 0.05m，400×400，重居中阈 0.25 | 已用 |
| 关键帧 | 0.5m / 15° / 2.0s（OR 触发） | 已用 |
| 回环（PC） | 半径 3.0m，间隔 20KF，阈值 0.6（CLI）/0.75（类默认）；似然场 10 环×0.85 衰减；粗搜 ±2m@0.2m/2°→精搜 ±0.1m/±2° | 仿真标定，实车待验 |
| 位姿图 | GN+LM(λ=1e-4)，iter≤10，Huber δ=0.8，回环边权=conf×1.5 | 仿真标定 |
| 电机/编码器引脚 | A(pwm18,23,24) B(pwm13,5,6)；编码器 A(17,27) B(22,10) | 接线见 `D:\桌面\毕设\接线.xlsx` |
| 视频 | 1280×720@30fps，x264 ultrafast zerolatency 2Mbps，GOP=30 | 链路已验证，CPU 占用【待验证】 |

## 8. 设计决策日志（新决策追加在末尾）

- **D1（历史 + 2026-09-28 作者确认）**：传输只用 TCP，**不再考虑 UDP**。历史原因：UDP 分片出过问题（git `51d7406`）；决定性原因：高频计算已全部放在树莓派本地，端云只需低频传输。8003/8004 UDP 端口仅作备用保留。
- **D2（2026-09-23）**：端口分配定稿，`43.227.71.58` 为**长期方案**【2026-09-28 作者确认】，旧 5001/5005 弃用。
- **D3（2026-09-23）**：`main.py --dry-run` 安全模式（传感器/SLAM/通信真实运行，电机禁用），用于台架联调。
- **D4（2026-09-23）**：PC 端 SLAM 后端重写——回环匹配用似然场+两级搜索（替换精确栅格命中），位姿图用 SE(2) 边+高斯牛顿（替换梯度下降，旧版仿真中发散至 ATE 4.3m）。仿真证据见 `tools/sim_loopclosure/`。
- **D5（2026-09-23）**：视频采用 libx264 软编码（本摄像头仅 MJPG 无原生 H264）；`--encode native` 保留给未来原生 H264 摄像头。
- **D6（2026-09-28 作者确认）**：**IMU 磁融合航向角（0x53 yaw）受干扰严重，计划弃用，只使用陀螺仪**。待实施改造：`slam/odometry.py` 弃用 `imu.latest_yaw`，改为 `latest_gyro_z` 积分（需处理零偏估计与漂移）。改造完成前 α=0.7 的互补滤波仍用 yaw。
- **D7（2026-09-28 作者确认）**：`control/car_controller.py` 标记遗留保留（不修不删）；手柄仅调试工具（不集成进 main.py）。
- **D8（2026-09-28 作者确认）**：无夜间巡检需求；实验场景≈20m 闭环；x264 CPU 占用未测。
- **D9（2026-09-28）**：知识库建立于两仓库 `docs/KNOWLEDGE_BASE.md`，随项目提交更新；Pi 仓库为系统级权威版所在。

## 9. 已知历史 Bug 档案（防再犯，含根因）

1. **帧头 12 vs 16 字节**：`!IQI`=4+8+4=16B。PC 端 `slam_client` 与 Pi 端 `udp_receiver` 先后各错一次（Pi 端 2026-09-23 修复，git `51538ea`）。教训：帧长应收敛为单一常量。
2. **PC 无头模式静默失联**：`--no-gui` 无消费者时 `local_map_queue` 满后阻塞 put 卡死接收线程，进程无任何报错。已改队满丢最旧（`_put_drop_oldest`）+ 主循环排空。
3. **预编译 .so 与 .pyx 源码不同步**：旧 `imu_parser.so` 无周期打包/flush，运行即崩（2026-09-23 重编译修复）。**slam 的 .so 从未入库**，Pi 当前跑纯 Python 回退（见 §10 R1）。
4. **`loop_detector` numpy 真值崩溃**：`if points_a`（ndarray）抛 ValueError——说明该匹配路径此前从未成功跑通过。
5. **扫描匹配先验坐标系 bug**：平移先验未旋转到候选帧体系，转角处匹配必错（已修）。
6. **Cython 编译目录陷阱**：setup 脚本必须从仓库根目录运行（内部拼绝对路径，从子目录跑 inplace 复制会失败——实测踩坑）。
7. **行尾陷阱**：Windows tar 直传会让 Pi 工作区变脏；部署后执行 `git config core.autocrlf false && git checkout -- .`。

## 10. 技术债务与风险登记（分级）

**确证（代码/环境可证）**
- **R1 slam 加速模块缺失**：仓库无 `slam/*.so`，Pi 跑纯 Python 回退，7 月"CPU 38%"优化未生效。恢复：Pi 上 `python3 slam/setup_scan.py build_ext --inplace`。
- **R2 无依赖清单**：两仓库均无 requirements.txt/pyproject.toml。
- **R3 PC 仓库无远程备份**：仅本地 git+tar；Pi 仓库提交需定期 push。
- **R4 文档漂移**：`SLAM_ARCHITECTURE.md`/`PC_DEBUG.md`/`PC/PROTOCOL_FIX.md` 仍是旧 UDP/旧端口描述；`CLAUDE.md` 部署章节引用已删除的 PC/ 目录。本知识库 + `DEPLOY.md` 为权威。

**高概率风险（强证据，未实车复现）**
- **R5**：局部子图未压缩 160KB/帧@1Hz≈1.3Mbps，与视频 2Mbps 并发总带宽未测（frp 免费服务器上限未知）。
- **R6**：位姿修正直接 reset，回环瞬间轨迹/地图跳变；局部地图内容不随修正迁移。
- **R7**：大漂移（>3m）超出回环匹配 ±2m 搜索窗会漏检（仿真 seed 7 实证）；自适应扩窗未实现（开题 3.3(2)）。
- **R8**：IMU yaw 磁干扰（作者确认）——D6 改造完成前，互补滤波的航向输入质量受限。

**观察项**
- Odometry 每 100Hz 触发 2 次 pigpio IPC 查询；编码器 EITHER_EDGE 无方向判别（倒车符号语义未验证）；`tcp_sender` 无应用层心跳（1Hz unified 实际充当数据面心跳）。

## 11. 测试现状与缺口

- **零自动化测试**（无 pytest/unittest/CI）。现有均为手动脚本：`imu/testimu.py`、`joy_test/test_joystick.py`、`camera/test.py|capture_test.py`、`*_profile.py`、`tools/link_check/`、`tools/sim_loopclosure/`。
- 最高优先补测点：`model/models.py` 六个数据类 pack/unpack round-trip（帧长历史出错两次）。
- 里程计/扫描匹配/建图无单测；仿真器只覆盖回环+位姿图。
- 数据记录/回放工具未实现（开题 3.2.5）。

## 12. 性能观察（推断，未实测量化）

| 位置 | 机制 | 缓解 |
|---|---|---|
| Pi·CSM（回退态） | O(21×21×31)×点数 纯 Python | 重建 .so 后 pyx 为粗细两级+180 点降采样 |
| Pi·x264 | 720p30 软编码 CPU 未测 | ultrafast+zerolatency 已选 |
| PC·回环匹配 | 每候选对 ~百万次查表（Python） | 低频触发可接受；只取前 50 点 |
| 网络 | 子图 1.3Mbps + 视频 2Mbps 并发未测 | — |

## 13. 路线图（对照开题报告）

| 开题条目 | 状态 |
|---|---|
| 3.2.1 里程计（互补滤波） | 基本完成；**标定待做；D6 陀螺仪改造待做** |
| 3.2.2 扫描匹配+局部建图 | 完成（含 Cython 优化设计）；`.so` 重建待做 |
| 3.2.3 端云回环+位姿图 | 检测/优化/回传完成；**平滑回注、全局地图融合、轻量压缩、（可选）学习型描述子待做** |
| 3.2.4 目标检测+语义建图 | 未开始 |
| 3.2.5 系统集成与实验 | 部分完成（视频/链路）；对比实验、数据记录回放未开始 |
| 3.2.6 语义地图+GLM 智能体 | 未开始 |
| 3.3(2) 自适应扩窗 | 未开始（R7） |

## 14. 常用操作手册

```bash
# ── 部署（PC → Pi）──
cd RaspberryPi-car
tar czf - --exclude=.venv --exclude=.idea --exclude=.claude --exclude=.continue \
    --exclude=__pycache__ --exclude=maps --exclude='*.pyc' --exclude=.zcodeignore . \
  | ssh -p 45704 raspicar@39.102.115.36 "tar xzf - -C ~/Desktop/raspberry_car"
ssh -p 45704 raspicar@39.102.115.36 \
  "cd ~/Desktop/raspberry_car && git config core.autocrlf false && git checkout -- ."
# Cython 重编译（必须仓库根目录）:
#   python3 imu/setup_imu.py build_ext --inplace
#   python3 lidar/setup.py build_ext --inplace
#   python3 slam/setup_scan.py build_ext --inplace   ← R1 恢复时执行

# ── 运行 ──
# Pi:  sudo systemctl start pigpiod && python3 main.py [--dry-run]
# PC:  .venv/Scripts/python.exe PC/slam_client.py [--no-gui]   （建议先于 Pi 启动）
#      .venv/Scripts/python.exe PC/video_receiver.py [--save F --duration S]

# ── 自检 ──
# PC:  .venv/Scripts/python.exe tools/link_check/pc_side.py --duration 55   （先启动）
# Pi:  python3 tools/link_check/pi_side.py --duration 40

# ── 仿真 ──
# PC:  .venv/Scripts/python.exe tools/sim_loopclosure/run_sim.py --seed 7

# ── 排错速查 ──
# Pi 每秒重连刷屏 → PC 端未监听: netstat -ano | findstr :8080
# 命令不通 → systemctl status frpc-zyfrp; journalctl -u frpc-zyfrp -n 20 (应有4条 start proxy success)
# 统一帧只有3子包 → IMU 掉线: 检查 /dev/ttyUSB0 与 imu_parser.so 版本
# PC 静默失联 → 见 §9.2（已修复，若复现检查队列消费）
```

### Git 约定（作者要求）
- 双仓库严格分离：端侧代码只进 `RaspberryPi-car`，云侧代码只进 `RaspberryPi-car-pc`；`model/models.py` 两边各留一份，修改时**必须同步且 diff 校验一致**。
- 中文提交信息、按逻辑分 commit；重要节点打 tag；阶段结束备份 tar.gz 到 `D:\桌面\毕设\backup_<日期>\`。
- Pi 仓库定期 `git push`（GitHub glglj/RaspberryPi-car）。
