import pyrealsense2 as rs
import numpy as np
import cv2

# 1. 카메라 설정 및 시작
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        # 2. 프레임 받아오기
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue

        # 3. 넘파이 배열로 변환 (OpenCV에서 쓰기 위함)
        color_image = np.asanyarray(color_frame.get_data())

        # 4. 화면에 띄우기 (이게 핵심!)
        cv2.imshow('RealSense Camera View', color_image)

        # 'q' 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
