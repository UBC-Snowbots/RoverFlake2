import pyrealsense2 as rs
import cv2
import subprocess
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

ffmpeg = subprocess.Popen(
    [
        "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24", "-s", "640x480", "-r", "30",
        "-i", "-", "-f", "mpegts", "udp://192.168.0.100:5000"
    ],
    stdin=subprocess.PIPE
)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        frame = np.asanyarray(color_frame.get_data())
        ffmpeg.stdin.write(frame.tobytes())
finally:
    pipeline.stop()
    ffmpeg.stdin.close()
    ffmpeg.wait()
