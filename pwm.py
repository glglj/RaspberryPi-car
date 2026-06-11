import pigpio


class Motor:
    """单路电机控制（L298N 双 H 桥芯片）"""

    def __init__(self, pi, pwm_pin, in1, in2, freq=1000):
        self.pi = pi
        self.pwm_pin = pwm_pin
        self.in1 = in1
        self.in2 = in2

        pi.set_mode(pwm_pin, pigpio.OUTPUT)
        pi.set_mode(in1, pigpio.OUTPUT)
        pi.set_mode(in2, pigpio.OUTPUT)

        pi.set_PWM_frequency(pwm_pin, freq)
        pi.set_PWM_dutycycle(pwm_pin, 0)

    def set_speed(self, speed):
        """speed: -100 ~ 100, 正=前进, 负=后退"""
        speed = max(-100, min(100, speed))
        duty = int(abs(speed) * 255 / 100)

        if speed > 0:
            self.pi.write(self.in1, 1)
            self.pi.write(self.in2, 0)
        elif speed < 0:
            self.pi.write(self.in1, 0)
            self.pi.write(self.in2, 1)
        else:
            self.pi.write(self.in1, 0)
            self.pi.write(self.in2, 0)
            self.pi.set_PWM_dutycycle(self.pwm_pin, 0)
            return

        self.pi.set_PWM_dutycycle(self.pwm_pin, duty)

    def stop(self):
        """惯性停止"""
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)
        self.pi.set_PWM_dutycycle(self.pwm_pin, 0)

    def brake(self):
        """急刹（IN1=IN2=1）"""
        self.pi.write(self.in1, 1)
        self.pi.write(self.in2, 1)
        self.pi.set_PWM_dutycycle(self.pwm_pin, 255)
