import cv2
import time

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# ⭐ 推荐先用30fps
cap.set(cv2.CAP_PROP_FPS, 30)

# ⭐ 曝光控制
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, -5)

# ⭐ 增益
cap.set(cv2.CAP_PROP_GAIN, 0)

# ⭐ 白平衡
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

time.sleep(1)

# ⭐ 丢帧稳定
for _ in range(20):
    cap.read()

ret, frame = cap.read()

if ret:
    cv2.imwrite("test.jpg", frame)
    print("✅ OK")
else:
    print("❌ FAIL")

cap.release()



