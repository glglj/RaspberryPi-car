import pigpio
from time import sleep

pi = pigpio.pi()
if not pi.connected:
    print("pigpio not running")
    exit()

APWM_PIN = 18   # PWMA
AIN1 = 23
AIN2 = 24

BPWM_PIN = 13   # PWMA
BIN1 = 5
BIN2 = 6

# 设置 GPIO 模式
pi.set_mode(APWM_PIN, pigpio.OUTPUT)
pi.set_mode(AIN1, pigpio.OUTPUT)
pi.set_mode(AIN2, pigpio.OUTPUT)

# 设置方向（正转）
pi.write(AIN1, 0)
pi.write(AIN2, 1)

pi.write(BIN1, 1)
pi.write(BIN2, 0)

# 设置 PWM 频率
pi.set_PWM_frequency(APWM_PIN, 1000)
pi.set_PWM_dutycycle(APWM_PIN, 10)

pi.set_PWM_frequency(BPWM_PIN, 1000)
pi.set_PWM_dutycycle(BPWM_PIN, 10)


