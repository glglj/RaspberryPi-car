import serial
import time

last_group_time = None
ser = serial.Serial('/dev/ttyUSB0', 9600)
buffer = bytearray()

def parse_frame(frame):
    # frame: 11字节
    global last_group_time
    header = frame[0]
    dtype  = frame[1]
    data   = frame[2:10]
    crc    = frame[10]
    if dtype == 0x51:
        now = time.time()

        if last_group_time is not None:
            dt = (now - last_group_time) * 1000  # ms
            print(f"📦 一组数据周期: {dt:.2f} ms")

        last_group_time = now
    # 计算校验
    calc_crc = (sum(frame[0:10]) & 0xFF)

    # 判断是否成功
    crc_ok = (calc_crc == crc)

    # 格式化输出
    data_str = ' '.join(f'0x{b:02X}' for b in data)

    result = "校验成功" if crc_ok else f"校验失败(计算:0x{calc_crc:02X} 实际:0x{crc:02X})"

    print(f"数据类型 0x{dtype:02X}  数据位 {data_str}  {result}")


while True:
    buffer += ser.read(64)

    while len(buffer) >= 11:
        # 找帧头
        if buffer[0] != 0x55:
            buffer.pop(0)
            continue

        # 类型检查（0x50~0x5F）
        if not (0x50 <= buffer[1] <= 0x5F):
            buffer.pop(0)
            continue

        frame = buffer[:11]

        parse_frame(frame)

        buffer = buffer[11:]