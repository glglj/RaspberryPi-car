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

    左摇杆 Y 轴 → 前进/后退
    右摇杆 X 轴 → 左转/右转
    Start 按钮 → 切换电机使能（安全锁，默认锁定）
    """

    # js_event 结构体: time(u32) + value(s16) + type(u8) + number(u8) = 8 bytes
    EVENT_FORMAT = "IhBB"
    EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02

    # 常见按钮编号 (Linux standard mapping)
    BTN_START = 9

    def __init__(self, motor_a, motor_b,
                 device="/dev/input/js0",
                 dead_zone=0.1,
                 max_speed=80):
        """
        Args:
            motor_a: Motor 实例（左轮）
            motor_b: Motor 实例（右轮）
            device: 手柄设备路径
            dead_zone: 死区比例 0.0~1.0，轴值绝对值 < 此值视为 0
            max_speed: 最大速度百分比 0~100，留余量保证安全
        """
        self.motor_a = motor_a
        self.motor_b = motor_b
        self.device = device
        self.dead_zone = dead_zone
        self.max_speed = max_speed

        self._axis_states = {}   # axis_number → value (-1.0 ~ 1.0)
        self._button_states = {} # button_number → 0/1
        self._enabled = False    # 电机使能标志（Start 按钮切换）

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
        print("[Joystick] 电机已锁定，按 Start 按钮解锁")

    def stop(self):
        """停止控制、关闭设备、停止电机"""
        self._running = False

        # 停止电机
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
        buf = b""
        while self._running:
            try:
                buf += self._js.read(self.EVENT_SIZE)
                if len(buf) < self.EVENT_SIZE:
                    continue

                while len(buf) >= self.EVENT_SIZE:
                    event_bytes = buf[:self.EVENT_SIZE]
                    buf = buf[self.EVENT_SIZE:]
                    self._process_event(event_bytes)

            except (OSError, struct.error):
                # 手柄拔出或其他读取错误
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
            pressed = value  # 0 或 1
            old = self._button_states.get(number, 0)
            self._button_states[number] = pressed

            # Start 按钮：上升沿切换使能
            if number == self.BTN_START and pressed and not old:
                self._enabled = not self._enabled
                state = "解锁" if self._enabled else "锁定"
                print(f"[Joystick] 电机{state}")

    def _apply_control(self):
        """差速驱动：将轴值映射到左右电机"""
        # 轴编号: 0=左X, 1=左Y, 2=右X, 3=右Y (标准映射)
        # 也兼容 axis 2/3 为 LT/RT 的手柄
        axis_y = self._axis_states.get(1, 0.0)  # 左摇杆 Y: 前进/后退
        axis_x = self._axis_states.get(2, 0.0)  # 右摇杆 X: 左转/右转

        # 如果没有右摇杆 X (axis 2)，尝试用左摇杆 X (axis 0)
        if axis_x == 0.0:
            axis_x = self._axis_states.get(0, 0.0)

        # 死区处理
        if abs(axis_y) < self.dead_zone:
            axis_y = 0.0
        if abs(axis_x) < self.dead_zone:
            axis_x = 0.0

        # 差速驱动: 左轮 = 前进 + 转向, 右轮 = 前进 - 转向
        forward = axis_y * self.max_speed  # Y轴：前推为正（-1.0=最前）
        turn = axis_x * self.max_speed

        left = self._clamp(forward + turn, -100, 100)
        right = self._clamp(forward - turn, -100, 100)

        self.motor_a.set_speed(left)
        self.motor_b.set_speed(right)

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))
