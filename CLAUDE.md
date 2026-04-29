# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Python 3 robotics project targeting a Raspberry Pi (aarch64) robot car. Integrates LIDAR sensing, motor encoder odometry, PWM motor control, and UDP telemetry to a remote command server. Code runs on the Pi; this repo is developed/edited on Windows.

## Running on Target (Raspberry Pi)

```bash
# Start main application
python3 main.py

# Build the Cython LIDAR parser (must be done on the Pi after source changes)
cd lidar && python3 setup.py build_ext --inplace

# Test LIDAR parser with pre-recorded data
python3 lidar/test.py

# Capture raw LIDAR data for testing
python3 lidar-catch.py

# Real-time LIDAR visualization (requires matplotlib)
python3 lidar/test_real.py
```

**Note:** `pigpiod` must be running (`sudo pigpiod`). `main.py` attempts to start it via `start_pigpio()` using `subprocess`.

## Architecture

### Component Relationships

```
main.py (orchestrator)
├── starts pigpiod daemon
├── initializes LidarSensor (lidar/lidar_to_PC.py)
│   └── uses lidar_parser (Cython .so) to parse UART frames from /dev/serial0 @ 230400 baud
├── initializes EncoderSensor (Encoder_to_PC.py)
│   └── uses pigpio edge callbacks on GPIO 17/22 to count encoder pulses
├── initializes UDPSender (udp_sender.py)
│   └── communicates with remote server bj.zyfrp.vip:5005
└── runs udp_loop() thread: polls sensors, sends telemetry, receives/dispatches commands
```

### UDP Protocol (custom binary)

All packets share an 8-byte header:
- Bytes 0-1: Magic `0xAA55` (little-endian)
- Bytes 2-3: Message type (little-endian)
- Bytes 4-7: Payload length (little-endian)

Message types:
- `0x01` — LIDAR frame: N × (angle float32, distance float32) pairs
- `0x02` — Encoder data: edge_A uint32, edge_B uint32
- `0x10` — Command poll (no payload)
- `0x11` — Command reply from server
- `0x12` — Command ACK

### LIDAR Parser (Performance-Critical)

`lidar/lidar_parser.pyx` is a Cython module compiled to a `.so` for speed. The `LidarParser.feed()` method parses the LIDAR binary protocol (sync `0xAA 0x55`, LSN data points, FSA/LSA angles, 3-byte point encoding). `LidarSensor.update()` accumulates packets until a full 360° scan is detected.

### GPIO Pin Assignments

| Function | GPIO |
|----------|------|
| Motor A PWM | 18 |
| Motor A IN1/IN2 | 23, 24 |
| Motor B PWM | 13 |
| Motor B IN1/IN2 | 5, 6 |
| Encoder A | 17 (main), 27 (opt) |
| Encoder B | 22 (main), 10 (opt) |

### Network

- Remote server: `bj.zyfrp.vip:5005` (UDP, hardcoded in `main.py`)
- SSH reverse tunnel via `frp_0.66.0_linux_arm/frpc` (Pi local port 22 → remote port 7777)

## Dependencies

System-level (not in venv, must be installed on Pi):
- `pigpio` + `pigpiod` daemon
- `pyserial`
- `cython` (build-time only)
- `matplotlib`, `numpy` (optional, for `test_real.py`)

The compiled extension `lidar/lidar_parser.cpython-311-aarch64-linux-gnu.so` targets Python 3.11 on aarch64. Rebuild on the Pi if Python version changes.

## Known Issues

- `main.py` has an incomplete/broken `udp_loop()` function with a `//` C-style comment that causes a `SyntaxError`. This needs to be fixed before the main entry point runs.
- Test data file `/home/pi/Desktop/raspberry_car/lidar_test.bin` referenced in `lidar/test.py` must exist on the Pi for that test to run.
- Server address and FRP credentials are hardcoded with no config file abstraction.