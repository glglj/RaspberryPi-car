# PC 端收不到数据 - 排查指南

## 当前通信架构

```
Pi (192.168.x.x)  ──TCP──→  frp server (bj.zyfrp.vip:5001)  ──frp tunnel──→  PC (localhost:5001)

Pi (192.168.x.x)  ←──TCP──  frp server (bj.zyfrp.vip:5006)  ←──frp tunnel──  PC
```

关键配置:
```python
# main.py (Pi)
TCP_SLAM_IP = "bj.zyfrp.vip"   # 发送目标
TCP_PORT = 5001                 # 发送端口
CMD_PORT = 5006                 # 接收端口 (Pi监听)
```

## 排查步骤（按顺序）

### 1. Pi 端是否连上了？

Pi 端应看到：
```
[TcpSender] 已连接到 ('bj.zyfrp.vip', 5001)
```

**没看到这条** → frp 隧道没通，检查 frp 配置。

**看到了但 PC 没收到** → 继续第 2 步。

### 2. 协议已变更，PC 端代码更新了吗？

Pi 端刚刚改过统一包子包头：**3 字节 → 5 字节**。PC 端必须同步更新 `slam_client.py` 的 `_handle_unified` 方法：

```python
# 旧（3字节头）
if offset + 3 > len(payload):
    break
sub_type = payload[offset]
sub_len = struct.unpack("!H", payload[offset + 1:offset + 3])[0]
offset += 3

# 新（5字节头）
if offset + 5 > len(payload):
    break
sub_type = payload[offset]
sub_len = struct.unpack("!I", payload[offset + 1:offset + 5])[0]
offset += 5
```

**没更新的表现**：PC 端可能打印 `[SlamClient] unified unpack error`，然后静默丢弃数据。

**关键帧不受影响**：`MSG_KEYFRAME` 是独立发送的，不走统一包。如果关键帧也收不到，说明 TCP 连接本身有问题。

### 3. PC 端端口对不对？

```bash
# PC 端启动命令（默认端口现在是 5001）
python3 PC/slam_client.py

# 确认监听
netstat -an | grep 5001    # Linux/Mac
netstat -an | findstr 5001 # Windows
```

应该看到 `0.0.0.0:5001 LISTENING`。

### 4. frp 隧道是否正常？

PC 端 frp 客户端需要把 `bj.zyfrp.vip:5001` 映射到本机 `5001`。

frpc.toml 示例（PC端）：
```toml
[[proxies]]
name = "slam_data"
type = "tcp"
localIP = "127.0.0.1"
localPort = 5001
remotePort = 5001
```

**验证 frp 连通性**：
```bash
# PC 上临时用 nc 监听，看能否收到原始字节
nc -l 5001 | xxd | head -20
```

如果 Pi 连上后 nc 有输出 → TCP 通，问题在代码。如果没输出 → frp / 网络问题。

### 5. 快速验证：直接 TCP 测试（不走 frp）

如果 Pi 和 PC 在同一局域网，先直连验证代码 OK：

```bash
# PC 上
python3 PC/slam_client.py --port 5001
```

```python
# Pi 上修改 main.py 临时改 IP：
TCP_SLAM_IP = "192.168.1.xxx"  # PC 的局域网 IP
```

跑起来看 PC 端是否收到数据。如果直连通、frp 不通 → 问题在 frp 配置。

### 6. 查看日志

PC 端正常应看到：
```
[SlamClient] Pi 已连接: ...
[PC] SLAM客户端已启动 (TCP), 监听端口 5001
```

如果有 parse error：
```
[SlamClient] unified unpack error: ...
[SlamClient] keyframe unpack error: ...
[SlamClient] local_map unpack error: ...
```

## 协议格式参考

```
外层帧 (12字节头):
| msg_type(4B BE) | timestamp(8B BE) | payload_len(4B BE) | payload(N bytes) |

统一包子包 (5字节头，刚改):
| sub_type(1B) | sub_len(4B BE) | sub_data(sub_len bytes) |

子类型:
  0x01 = MSG_LIDAR
  0x03 = MSG_IMU
  0x04 = MSG_ODOM
  0x06 = MSG_LOCAL_MAP
```
