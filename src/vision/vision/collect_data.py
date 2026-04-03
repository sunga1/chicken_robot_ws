#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time
from tf2_ros import Buffer, TransformListener
from dsr_msgs2.srv import MoveLine

class ChickenDataCollector(Node):
    def __init__(self):
        super().__init__('chicken_data_collector')
        
        # [1] 저장 경로 및 서비스 설정
        self.save_path = os.path.expanduser('~/chicken_robot_ws/src/vision/data/images')
        os.makedirs(self.save_path, exist_ok=True)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_cli = self.create_client(MoveLine, '/motion/move_line')

        # [2] 리얼센스 고해상도 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 해상도 1280x720
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
        
        # [3] 선명도를 위한 후처리 필터 추가
        self.colorizer = rs.colorizer()
        self.align = rs.align(rs.stream.color)
        
        # 하드웨어 설정: 선명도(Sharpness)와 노출(Exposure) 최적화
        self.profile = self.pipeline.start(config)
        sensor = self.profile.get_device().query_sensors()[1] # Color Sensor
        sensor.set_option(rs.option.sharpness, 100) # 선명도 최대 (0~100)
        sensor.set_option(rs.option.enable_auto_exposure, 1) # 자동 노출 켬

        self.current_img = None
        self.move_step = 10.0
        self.create_timer(0.1, self.main_loop)
        self.get_logger().info(" 고해상도(1280x720) 모드")

    def move_robot(self, dx=0.0, dy=0.0, dz=0.0):
        """현재 위치를 기준으로 로봇을 상대 이동시킴"""
        try:
            # TF를 통해 현재 위치(base_link -> link_6) 조회
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'link_6', now, rclpy.duration.Duration(seconds=0.1))
            
            req = MoveLine.Request()
            # 현재 좌표(m -> mm 변환) + 이동량(mm)
            req.pos = [
                (trans.transform.translation.x * 1000.0) + dx,
                (trans.transform.translation.y * 1000.0) + dy,
                (trans.transform.translation.z * 1000.0) + dz,
                0.0, 180.0, 0.0  # 작업 각도 고정
            ]
            req.vel, req.acc = [100.0, 100.0], [100.0, 100.0]
            
            self.move_cli.call_async(req)
        except Exception:
            pass

    def main_loop(self):
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: return

            # [4] 넘파이 배열 변환 시 고품질 유지
            img = np.asanyarray(color_frame.get_data())
            
            self.current_img = img
            display_img = cv2.resize(img, (640, 360)) 
            
            cv2.imshow("Chicken Data Collector (Preview)", display_img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('w'): self.move_robot(dx=self.move_step)
            elif key == ord('s'): self.move_robot(dx=-self.move_step)
            elif key == ord('a'): self.move_robot(dy=self.move_step)
            elif key == ord('d'): self.move_robot(dy=-self.move_step)
            elif key == ord('c'): self.save_image()
            elif key == ord('q'): rclpy.shutdown()

        except Exception: pass
        
    def save_image(self):
        """고해상도 원본 저장"""
        if self.current_img is not None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"high_res_{timestamp}.jpg"
            full_path = os.path.join(self.save_path, filename)
            
            cv2.imwrite(full_path, self.current_img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            self.get_logger().info(f"이미지 저장: {filename}")

def main():
    rclpy.init()
    node = ChickenDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.pipeline.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
