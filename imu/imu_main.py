from  imu.imu import IMUSensor

imu = IMUSensor("/dev/ttyUSB0", 115200)

imu.start()

while True:
    data = imu.get()

    print(data)