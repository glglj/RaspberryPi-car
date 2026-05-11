from  imu.Imu import IMUSensor

imu = IMUSensor("/dev/ttyUSB0", 9600)

imu.start()

while True:
    data = imu.get()

    print(data)