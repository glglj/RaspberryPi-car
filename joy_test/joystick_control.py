"""
无线手柄控制模块

读取 Linux 标准手柄设备 /dev/input/js0，使用差速驱动算法将摇杆映射到电机控制。
纯 Python + struct 实现，零外部依赖。
"""

import struct
import threading
import time


class JoystickController:
    """手柄差速控制小车

    默认左摇杆控制：Y轴 → 前进/后退，X轴 → 左转/右转
    Start 按钮 → 切换电机使能（安全锁，默认锁定）

    可通过 axis_y/axis_x 参数调整轴映射，适配不同手柄。
    """

    # js_event 结构体: time(u32) + value(s16) + type(u8) + number(u8) = 8 bytes
    EVENT_FORMAT = "IhBB"
    EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02

    BTN_START = 9

    def __init__(self, motor_a, motor_b,
                 device="/dev/input/js0",
                 dead_zone=0.1,
                 max_speed=80,
                 axis_y=1,
                 axis_x=0,
                 invert_y=True):
        """
        Args:
            motor_a: Motor 实例（左轮）
            motor_b: Motor 实例（右轮）
            device: 手柄设备路径
            dead_zone: 死区比例 0.0~1.0，轴值绝对值 < 此值视为 0
            max_speed: 最大速度百分比 0~100
            axis_y: 前进/后退轴编号 (默认 1 = 左摇杆Y)
            axis_x: 转向轴编号 (默认 0 = 左摇杆X)
            invert_y: Linux手柄前推为负值，设为True自动反转
        """
        self.motor_a = motor_a
        self.motor_b = motor_b
        self.device = device
        self.dead_zone = dead_zone
        self.max_speed = max_speed
        self.axis_y = axis_y
        self.axis_x = axis_x
        self.invert_y = invert_y

        self._axis_states = {}
        self._button_states = {}
        self._enabled = False

        self._js = None
        self._running = False
        self._read_thread = None
        self._control_thread = None

    # ─── 公开 API ───────────────────────────────────────────

    def start(self):
        """启动后台线程：读取手柄事件 + 控制循环"""
        if self._running:
            return

        self._js = open(self.device, "rb")
        self._running = True

        self._read_thread = threading.Thread(
            target=self._read_loop, name="js-read", daemon=True)
        self._control_thread = threading.Thread(
            target=self._control_loop, name="js-ctrl", daemon=True)

        self._read_thread.start()
        self._control_thread.start()

        print(f"[Joystick] 已连接 {self.device}")
        print(f"[Joystick] 轴映射: 前进/后退=axis{self.axis_y}, 转向=axis{self.axis_x}")
        print("[Joystick] 电机已锁定，按 Start 按钮解锁")

    def stop(self):
        """停止控制、关闭设备、停止电机"""
        self._running = False

        try:
            self.motor_a.stop()
            self.motor_b.stop()
        except Exception:
            pass

        if self._js:
            try:
                self._js.close()
            except Exception:
                pass
            self._js = None

        print("[Joystick] 已停止")

    # ─── 后台线程 ───────────────────────────────────────────

    def _read_loop(self):
        """后台线程：持续读取手柄事件，更新轴值和按钮状态"""
        while self._running:
            try:
                data = self._js.read(self.EVENT_SIZE)
                if len(data) < self.EVENT_SIZE:
                    continue
                self._process_event(data)
            except (OSError, struct.error):
                print("[Joystick] 读取错误，设备可能已拔出，停止电机")
                self._running = False
                try:
                    self.motor_a.stop()
                    self.motor_b.stop()
                except Exception:
                    pass
                break
            except Exception:
                break

    def _control_loop(self):
        """后台线程：50Hz 将轴状态映射为电机 PWM"""
        interval = 1.0 / 50
        while self._running:
            if self._enabled:
                self._apply_control()
            time.sleep(interval)

    # ─── 内部逻辑 ───────────────────────────────────────────

    def _process_event(self, data):
        """解析单个 js_event，更新内部状态"""
        _time, value, etype, number = struct.unpack(self.EVENT_FORMAT, data)

        if etype & self.JS_EVENT_AXIS:
            # 归一化到 -1.0 ~ 1.0
            normalized = value / 32767.0
            self._axis_states[number] = normalized

        elif etype & self.JS_EVENT_BUTTON:
            pressed = value
            old = self._button_states.get(number, 0)
            self._button_states[number] = pressed

            # Start 按钮：上升沿切换使能
            if number == self.BTN_START and pressed and not old:
                self._enabled = not self._enabled
                if self._enabled:
                    self._print_axis_diag()
                    print("[Joystick] 电机解锁")
                else:
                    # 锁定时立即停转
                    self.motor_a.stop()
                    self.motor_b.stop()
                    print("[Joystick] 电机锁定")

    def _print_axis_diag(self):
        """解锁时打印所有轴当前值，帮助排查映射问题"""
        if not self._axis_states:
            print("[Joystick] 未检测到轴事件，请推动摇杆后重试")
            return
        print("[Joystick] 当前轴值(归一化):")
        for num in sorted(self._axis_states):
            val = self._axis_states[num]
            marker = ""
            if num == self.axis_y:
                marker = "  ← 前进/后退"
            elif num == self.axis_x:
                marker = "  ← 转向"
            print(f"  axis {num}: {val:+7.4f}{marker}")

    def _apply_control(self):
        """差速驱动：将轴值映射到左右电机"""
        raw_y = self._axis_states.get(self.axis_y, 0.0)
        raw_x = self._axis_states.get(self.axis_x, 0.0)

        # 死区
        y = 0.0 if abs(raw_y) < self.dead_zone else raw_y
        x = 0.0 if abs(raw_x) < self.dead_zone else raw_x

        # Linux 手柄前推为负值，反转后：前推 = 正速度
        forward = (-y if self.invert_y else y) * self.max_speed
        turn = x * self.max_speed

        left = self._clamp(forward + turn, -100, 100)
        right = self._clamp(forward - turn, -100, 100)

        self.motor_a.set_speed(left)
        self.motor_b.set_speed(right)

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))
