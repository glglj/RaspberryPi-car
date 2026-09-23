"""
摄像头本地录制测试 - 抓取 H264 流保存到文件

用法:
    python camera/capture_test.py            # 录制 5 秒，保存到 test.h264
    python camera/capture_test.py --time 10  # 录制 10 秒
    python camera/capture_test.py --list     # 列出视频设备
"""

import subprocess
import argparse
import os
import time


def list_devices():
    """列出 /dev/video* 设备"""
    print("=== 检查视频设备 ===")
    # 检查 /dev/video*
    for i in range(4):
        dev = f"/dev/video{i}"
        if os.path.exists(dev):
            print(f"  存在: {dev}")
            # 用 v4l2-ctl 查看详细信息
            try:
                result = subprocess.run(
                    ["v4l2-ctl", "--device", dev, "--list-formats-ext"],
                    capture_output=True, text=True, timeout=5)
                print(f"  {result.stdout[:500]}")
            except FileNotFoundError:
                print("  (v4l2-ctl 未安装)")
            except Exception as e:
                print(f"  查询失败: {e}")
        else:
            print(f"  不存在: {dev}")

    # 检查 ffmpeg
    print("\n=== 检查 ffmpeg ===")
    try:
        result = subprocess.run(["ffmpeg", "-version"], capture_output=True,
                                text=True, timeout=5)
        print(f"  ffmpeg 可用: {result.stdout.split(chr(10))[0]}")
    except FileNotFoundError:
        print("  ffmpeg 未找到!")


def capture_test(output_file, duration, width=1280, height=720, fps=30):
    """录制一段 H264 视频到本地文件"""
    print(f"\n=== 录制测试 ===")
    print(f"  设备: /dev/video0")
    print(f"  分辨率: {width}x{height} @{fps}fps")
    print(f"  时长: {duration}s")
    print(f"  输出: {output_file}")

    if not os.path.exists("/dev/video0"):
        print("\n错误: /dev/video0 不存在!")
        print("请检查摄像头是否已连接")
        return False

    cmd = [
        "ffmpeg",
        "-f", "v4l2",
        "-input_format", "h264",
        "-video_size", f"{width}x{height}",
        "-framerate", str(fps),
        "-i", "/dev/video0",
        "-c", "copy",               # 透传不重编码
        "-t", str(duration),        # 录制时长
        "-f", "h264",
        "-y",                        # 覆盖已有文件
        output_file
    ]

    print(f"\n命令: {' '.join(cmd)}\n")

    start = time.time()
    proc = subprocess.Popen(cmd, stderr=subprocess.PIPE, text=True)
    proc.wait(timeout=duration + 10)
    elapsed = time.time() - start

    if proc.returncode == 0 and os.path.exists(output_file):
        size = os.path.getsize(output_file)
        print(f"\n录制成功! 耗时={elapsed:.1f}s, "
              f"文件={output_file}, 大小={size/1024:.1f} KB")
        if size > 0:
            kbps = (size * 8) / (duration * 1000)
            print(f"码率约: {kbps:.0f} kbps")
        print(f"\n播放测试: ffplay {output_file}")
        return True
    else:
        print(f"\n录制失败! ffmpeg 返回码={proc.returncode}")
        stderr_out = proc.stderr.read() if proc.stderr else ""
        if stderr_out:
            # 只打印最后几行
            lines = stderr_out.strip().split('\n')
            for line in lines[-10:]:
                print(f"  {line}")
        return False


def main():
    parser = argparse.ArgumentParser(description="摄像头本地录制测试")
    parser.add_argument("--list", action="store_true", help="列出视频设备")
    parser.add_argument("--time", type=int, default=5, help="录制时长(秒)")
    parser.add_argument("--output", default="test.h264", help="输出文件名")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    if args.list:
        list_devices()
        return

    success = capture_test(args.output, args.time,
                           args.width, args.height, args.fps)

    if not success:
        print("\n=== 故障排查 ===")
        print("1. 检查摄像头是否插好: ls /dev/video*")
        print("2. 检查摄像头是否支持 H264: v4l2-ctl --device /dev/video0 --list-formats-ext")
        print("3. 尝试用 --list 参数查看设备信息")
        print("4. 确认没有其他程序占用摄像头")


if __name__ == "__main__":
    main()
