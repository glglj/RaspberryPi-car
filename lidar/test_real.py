import serial
import matplotlib.pyplot as plt
import numpy as np
from lidar_parser import LidarParser

parser = LidarParser()

uart = serial.Serial(
    port="/dev/serial0",  # GPIO UART
    baudrate=230400,  # 雷达波特率
    timeout=0.1
)


plt.ion()
fig = plt.figure(figsize=(6, 6))
ax = plt.subplot(111, projection='polar')
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_rmax(20000)   # 按雷达量程调整

sc = ax.scatter([], [], s=5)

angles = []
dists = []

print("开始实时绘图")

while True:
    data = uart.read(4096)
    if not data:
        plt.pause(0.001)
        continue

    pkgs = parser.feed(data)

    for pkg in pkgs:
        for si in pkg["Si"]:
            angles.append(np.deg2rad(si["angle"]))
            dists.append(si["distance"])

    # 控制点数，防止无限增长
    if len(angles) > 2000:
        angles = angles[-2000:]
        dists = dists[-2000:]

    sc.set_offsets(np.c_[angles, dists])
    plt.pause(0.001)
