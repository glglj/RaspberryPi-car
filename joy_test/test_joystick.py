#!/usr/bin/env python3
"""
手柄控制小车 - 独立测试脚本

直接测试手柄控制功能，不启动雷达/IMU/UDP 等模块。

用法:
    python3 test_joystick.py [/dev/input/js0]

    Ctrl+C 停止，电机会自动停转。
"""

import sys
import signal
import pigpio
from pwm import Motor
from joy_test.joystick_control import JoystickController

DEVICE = sys.argv[1] if len(sys.argv) > 1 else "/dev/input/js0"


def main():
    # 连接 pigpio 守护进程
    pi = pigpio.pi()
    if not pi.connected:
        print("错误: 无法连接 pigpio 守护进程，请先运行 sudo pigpiod")
        return

    # 初始化双路电机
    motor_a = Motor(pi, pwm_pin=18, in1=23, in2=24, freq=1000)
    motor_b = Motor(pi, pwm_pin=13, in1=5,  in2=6,  freq=1000)

    # 创建手柄控制器
    joystick = JoystickController(motor_a, motor_b, device=DEVICE,
                                  axis_y=0, axis_x=1)

    # 注册 Ctrl+C 清理
    def cleanup(signum, frame):
        print("\n正在停止...")
        joystick.stop()
        motor_a.stop()
        motor_b.stop()
        pi.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # 启动
    joystick.start()

    try:
        print("按 Ctrl+C 退出")
        signal.pause()
    except KeyboardInterrupt:
        pass
    finally:
        cleanup(None, None)


if __name__ == "__main__":
    main()
