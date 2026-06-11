import threading
import time
from model.models import CMD_STOP, CMD_STRAIGHT, CMD_TURN_LEFT, CMD_TURN_RIGHT


class StraightController:
    """直行控制器：保持左右轮编码器增量一致（比例控制）"""

    def __init__(self, motor_a, motor_b, encoder):
        self.motor_a = motor_a
        self.motor_b = motor_b
        self.encoder = encoder
        self._kp = 0.5

    def control(self, base_speed):
        """一次控制迭代，base_speed: -100 ~ 100"""
        frame = self.encoder.read()
        left_edges = frame.a_edges
        right_edges = frame.b_edges

        error = left_edges - right_edges
        left_pwm = base_speed - self._kp * error
        right_pwm = base_speed + self._kp * error

        # 限制 PWM 范围
        left_pwm = max(-100, min(100, left_pwm))
        right_pwm = max(-100, min(100, right_pwm))

        self.motor_a.set_speed(left_pwm)
        self.motor_b.set_speed(right_pwm)


class TurnController:
    """转向控制器：IMU yaw 角度闭环差速转向"""

    def __init__(self, motor_a, motor_b, imu, turn_speed=40, threshold=2.0):
        self.motor_a = motor_a
        self.motor_b = motor_b
        self.imu = imu
        self.turn_speed = turn_speed
        self.threshold = threshold

        self._start_yaw = 0.0
        self._accumulator = 0.0  # 累积转角

    def start_turn(self, direction, target_angle):
        """开始转向：direction = 'left' | 'right'"""
        self._direction = direction
        self._target_angle = target_angle
        self._start_yaw = self.imu.latest_yaw
        self._accumulator = 0.0
        self._prev_yaw = self._start_yaw

    def control(self):
        """一次控制迭代，返回 True 表示转向完成"""
        current_yaw = self.imu.latest_yaw

        # 计算 yaw 增量（处理 ±180 边界跳变）
        delta = ((current_yaw - self._prev_yaw + 180) % 360) - 180
        self._accumulator += delta
        self._prev_yaw = current_yaw

        abs_accum = abs(self._accumulator)
        error = self._target_angle - abs_accum

        if error <= self.threshold:
            self.motor_a.stop()
            self.motor_b.stop()
            return True

        # 接近目标时减速
        speed = self.turn_speed
        if error < self.threshold * 3:
            speed = max(10, self.turn_speed // 2)

        if self._direction == "left":
            self.motor_a.set_speed(speed)
            self.motor_b.set_speed(-speed)
        else:
            self.motor_a.set_speed(-speed)
            self.motor_b.set_speed(speed)

        return False


class MotionController:
    """运动控制器：直行 + 转向，50Hz 控制线程"""

    def __init__(self, motor_a, motor_b, encoder, imu):
        self._straight = StraightController(motor_a, motor_b, encoder)
        self._turn = TurnController(motor_a, motor_b, imu)

        self._lock = threading.Lock()
        self._cmd_type = CMD_STOP
        self._param = 0.0
        self._running = False
        self._thread = None

    # ---- 线程安全的外部接口 ----

    def go_straight(self, speed: int):
        """直线行驶，speed: -100 ~ 100"""
        with self._lock:
            self._cmd_type = CMD_STRAIGHT
            self._param = speed

    def turn(self, direction: str, angle: float):
        """转向：direction = 'left' | 'right'，angle 单位度"""
        cmd = CMD_TURN_LEFT if direction == "left" else CMD_TURN_RIGHT
        with self._lock:
            self._cmd_type = cmd
            self._param = angle

    def stop(self):
        """停止所有运动"""
        with self._lock:
            self._cmd_type = CMD_STOP
            self._param = 0.0

    # ---- 控制线程 ----

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop_controller(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self._straight.motor_a.stop()
        self._straight.motor_b.stop()

    def _worker(self):
        period = 1.0 / 50  # 50 Hz
        turn_active = False
        straight_active = False

        while self._running:
            with self._lock:
                cmd = self._cmd_type
                param = self._param

            if cmd == CMD_STOP:
                if straight_active or turn_active:
                    self._straight.motor_a.stop()
                    self._straight.motor_b.stop()
                    straight_active = False
                    turn_active = False

            elif cmd == CMD_STRAIGHT:
                if not straight_active:
                    turn_active = False
                    straight_active = True
                self._straight.control(param)

            elif cmd in (CMD_TURN_LEFT, CMD_TURN_RIGHT):
                if not turn_active:
                    straight_active = False
                    turn_active = True
                    direction = "left" if cmd == CMD_TURN_LEFT else "right"
                    self._turn.start_turn(direction, param)

                done = self._turn.control()
                if done:
                    turn_active = False
                    with self._lock:
                        self._cmd_type = CMD_STOP
                        self._param = 0.0

            time.sleep(period)
