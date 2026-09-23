# 端云协同系统部署文档

> 树莓派端 (RaspberryPi-car) ↔ PC端 (RaspberryPi-car-pc) 通过 frp 内网穿透互联。
> 本文记录通信链路端口分配、部署步骤与端到端联调方法。
> 最后更新: 2026-09-23

## 1. 通信链路端口分配

### 已验证的最终分配 (2026-09-23 联调通过)

| 链路 | 公网端点 | 落地 | 用途 | 提供方 |
|------|----------|------|------|--------|
| **Pi→PC 数据** | `43.227.71.58:44310` (TCP) | PC `127.0.0.1:8080` | SLAM统一数据流 (雷达/IMU/里程计/局部子图/关键帧, 1Hz MSG_UNIFIED + 事件帧) | PC frpmgr 映射 `44310→8080` |
| **Pi→PC 视频** | `43.227.71.58:42011` (TCP) | PC `127.0.0.1:8888` | H264视频流 (本摄像头仅MJPG, 树莓派端libx264软编码 2Mbps) | PC frpmgr 映射 `42011→8888` |
| **PC→Pi 命令** | `bj.zyfrp.vip:8002` (TCP) | Pi `127.0.0.1:8002` | 运动指令 / 回环约束 / 位姿修正 | Pi 端 `frpc-zyfrp` 服务 映射 `8002→8002` |
| Pi侧预留 | `bj.zyfrp.vip:8001` (TCP) | Pi `8001` | 预留 (备用命令/调试通道) | 同上 |
| Pi侧预留 | `bj.zyfrp.vip:8003` (UDP) | Pi `8003` | 预留 (低延迟遥控/遥测, 见开题报告 3.3(3)) | 同上 |
| Pi侧预留 | `bj.zyfrp.vip:8004` (UDP) | Pi `8004` | 预留 | 同上 |

### 备用/失效端口说明

| 端点 | 状态 | 说明 |
|------|------|------|
| `bj.zyfrp.vip:5001` → PC:5001 | **当前不可用** | 该 frps (62.234.222.121:899) 拒绝连接, frpmgr 持续重试中。恢复后可改回 (Pi端 `main.py: TCP_SLAM_IP/TCP_PORT`) |
| `bj.zyfrp.vip:5005` (UDP) → PC:5005 | 同上 | 预留 UDP 遥测通道 |
| Pi SSH | `39.102.115.36:45704` | Pi 端 `frpc.service` (系统服务, /opt/frp/frpc.toml), 公钥登录 |

## 2. frp 拓扑

```
树莓派 (raspberrypi)                          PC (Windows)
┌─────────────────────────┐                ┌──────────────────────────┐
│ frpc.service            │                │ frpmgr (D:/frp)          │
│  /opt/frp/frpc.toml     │──→ 39.102.115.36:7000   │  43.227.71.58.toml       │
│  → SSH 22 → 45704       │                │   44310 → 8080 (数据)    │
│                         │                │   42011 → 8888 (视频)    │
│ frpc-zyfrp.service  ★新 │──→ bj.zyfrp.vip:898   │  (bj.zyfrp.vip.toml      │
│  /opt/frp/frpc_zyfrp.toml│                │   5001/5005 映射当前不可用)│
│  → 8001/8002/8003/8004  │                │                          │
└─────────────────────────┘                └──────────────────────────┘
```

- 四个新端口 (`8001-8004`, zyfrp 账号 1582508187) **全部留给树莓派端**, 配置文件
  `/opt/frp/frpc_zyfrp.toml` 由本目录 `frp配置.txt` 部署而来。
- PC 端不再新增映射, Pi→PC 全部复用 frpmgr 已有映射 (本机已映射端口/)。

## 3. 部署步骤

### 3.1 树莓派端 (代码: RaspberryPi-car)

```bash
# PC 上打包推送 (Git Bash)
cd RaspberryPi-car
tar czf - --exclude=.venv --exclude=.idea --exclude=.claude \
    --exclude=.continue --exclude=__pycache__ --exclude=maps \
    --exclude='*.pyc' --exclude=.zcodeignore . \
  | ssh -p 45704 raspicar@39.102.115.36 "tar xzf - -C ~/Desktop/raspberry_car"
ssh -p 45704 raspicar@39.102.115.36 \
  "cd ~/Desktop/raspberry_car && git config core.autocrlf false && git checkout -- ."
# (checkout 仅用于归一化 Windows CRLF 行尾, 不会丢失内容)

# 树莓派上: Cython 模块如与源码不一致需重编译 (一般无需, 仓库已带 aarch64 预编译 .so)
cd ~/Desktop/raspberry_car
python3 imu/setup_imu.py build_ext --inplace     # 注意: 必须在仓库根目录执行
python3 lidar/setup.py build_ext --inplace
```

### 3.2 PC 端 (代码: RaspberryPi-car-pc)

```bash
cd RaspberryPi-car-pc
.venv/Scripts/python.exe PC/slam_client.py            # 带可视化
.venv/Scripts/python.exe PC/slam_client.py --no-gui   # 无头模式
.venv/Scripts/python.exe PC/video_receiver.py         # 视频接收 (监听 8888)
```

## 4. 启动顺序

```bash
# ── 树莓派 ──
sudo systemctl start pigpiod          # GPIO 守护进程
python3 main.py --dry-run             # 台架联调: 传感器+SLAM+通信正常, 电机禁用 ★
python3 main.py                       # 正常运行 (电机使能)

# ── PC ──
.venv/Scripts/python.exe PC/slam_client.py            # 先于树莓派启动更稳
.venv/Scripts/python.exe PC/video_receiver.py         # 需要视频时 (PC 需装有 ffmpeg/ffplay)
```

> 服务自启: Pi 端 `frpc.service`(SSH) 与 `frpc-zyfrp.service`(8001-8004) 均
> 已 `systemctl enable`, 开机自动拉起。

## 5. 视频链路自检 (2026-09-23 已验证)

```bash
# 树莓派: 推流 (MJPG摄像头需 libx264 软编码)
python3 camera/h264_streamer.py --host 43.227.71.58 --port 42011     --encode libx264 --width 1280 --height 720 --fps 30

# PC: 录制 8 秒验证 (或去掉 --save/--duration 用 ffplay 实时观看)
.venv/Scripts/python.exe PC/video_receiver.py --port 8888     --save video_test.h264 --duration 8
ffprobe video_test.h264   # 应显示 h264, 1280x720
```
注: 已验证全链路 (MJPG采集→x264编码→frp隧道→PC录制→ffprobe解码)。
夜间无光时画面为黑/噪点属正常, 开灯即可。

## 6. 链路自检 (无需传感器/电机)

```bash
# PC (先启动, 监听 8080 + 连接 8002)
cd RaspberryPi-car-pc
.venv/Scripts/python.exe tools/link_check/pc_side.py --duration 55

# 树莓派 (40 秒内启动, 连 44310 + 监听 8002)
cd ~/Desktop/raspberry_car
python3 tools/link_check/pi_side.py --duration 40
```
两侧均输出 `自检结论: PASS` 即双向链路正常。

## 7. 故障排查

| 现象 | 排查 |
|------|------|
| Pi 端每秒重连 (`已连接到` 刷屏) | PC 端 slam_client 未启动/监听端口不对; `netstat -ano | findstr :8080` |
| PC 收不到数据 | frpmgr 是否在跑; `43.227.71.58:44310` 是否可达 (`socket.create_connection`); bj.zyfrp.vip:899 上的 5001 映射服务器曾整段不可用, 已改用 43.227.71.58 |
| 命令不通 | Pi 端 `systemctl status frpc-zyfrp`; `journalctl -u frpc-zyfrp -n 20` 应有 4 条 `start proxy success` |
| IMU 缺失 (统一帧 3 子包) | imu_parser.so 与 .pyx 版本不一致 (历史坑: 旧 .so 无 bundle/flush), 重编译 3.1 节命令 |
| PC 静默失联 (无任何断开日志) | 历史 bug: --no-gui 无消费者时 `local_map_queue.put` 阻塞卡死接收线程, 已改为队满丢最旧 + 主循环排空 (slam_client.py `_put_drop_oldest`) |

## 8. 联调记录 (2026-09-23)

- 双向链路自检 PASS: Pi→PC 80 帧合成里程计全收; PC→Pi 位姿修正 + CMD_STOP 正确解析。
- 真实系统 dry-run 联调 PASS: Pi 端雷达/IMU/编码器 + SLAM (栅格 400×400@5cm 持续更新,
  关键帧 0.5Hz 时间触发) → frp → PC 端位姿图 (关键帧计数与图节点持续增长), 连接稳定无重连。
- 视频链路验证: Pi 摄像头 (MJPG 1280x720) libx264 软编码 → 43.227.71.58:42011
  → PC:8888 录制 8s (335KB), ffprobe 确认 h264/1280x720 可正常解码
- 修复: Pi 端命令帧头 12→16 字节 (`robot_run/udp_receiver.py`);
  PC 端无头模式队列阻塞 (`PC/slam_client.py`); imu_parser.so 与源码不一致 (重编译)。
