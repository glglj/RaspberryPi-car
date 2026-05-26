import subprocess
import numpy as np
import cv2

w = 1280
h = 720

cmd = [
    'ffmpeg',
    '-f', 'v4l2',
    '-input_format', 'h264',
    '-video_size', f'{w}x{h}',
    '-i', '/dev/video0',
    '-f', 'rawvideo',
    '-pix_fmt', 'bgr24',
    '-'
]

pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE)

raw = pipe.stdout.read(w * h * 3)

frame = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))

cv2.imwrite("/home/pi/test.jpg", frame)

pipe.kill()