import time
from lidar_parser import LidarParser

parser = LidarParser()
total_bytes = 0

start_time = time.time()

with open("/home/pi/Desktop/raspberry_car/lidar_test.bin", "rb") as f:
    while True:
        data = f.read(4096)
        if not data:
            break
        total_bytes += len(data)
        res = parser.feed(data)

for pkg in res:
    print("包起始索引:", pkg['start_idx'], "LSN:", pkg['LSN'],
          "FSA_raw:", pkg['FSA_raw'], "LSA_raw:", pkg['LSA_raw'])
    for idx, si in enumerate(pkg['Si']):
        print(f"  点 {idx}: angle={si['angle']:.2f}, distance={si['distance']}, intensity={si['intensity']}")

end_time = time.time()
print(f"总数据量: {total_bytes} bytes")
print(f"处理时间: {end_time - start_time:.4f} s")
print(f"吞吐率: {total_bytes / (end_time - start_time) / 1024:.2f} KB/s")
