#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge # 💡 꼭 설치되어 있어야 합니다!
from ultralytics import YOLO

class ChickenDetector(Node):
    def __init__(self):
        super().__init__('chicken_detector')
        
        # 1. YOLO 모델 로드 (성아님의 경로: /home/sunga/Downloads/best.pt)
        try:
            self.model = YOLO('/home/sunga/Downloads/best.pt')
            self.get_logger().info("✅ 모델 로드 성공!")
        except Exception as e:
            self.get_logger().error(f"❌ 모델 로드 실패: {e}")

        # 2. ROS 2 <-> OpenCV 변환 도구
        self.bridge = CvBridge()

        # 3. 리얼센스 컬러 영상 구독
        # (만약 영상이 안 뜨면 아래 토픽 이름을 'ros2 topic list'로 확인해 보세요)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw', # 💡 토픽 이름이 정확한지 꼭 확인!
            self.image_callback,
            10)

        self.get_logger().info("🚀 치킨 탐지 노드가 시작되었습니다! 화면을 확인하세요.")

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV(BGR)로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # YOLOv8 추론 (confidence 0.5 이상만, verbose=False로 터미널 깨끗하게)
            results = self.model.predict(frame, conf=0.5, verbose=False)

            # 결과 시각화 (YOLO 내장 기능 사용)
            annotated_frame = results[0].plot()

            # 📺 화면 띄우기 (이 부분이 핵심!)
            cv2.imshow("Chicken Detection", annotated_frame)
            
            # 💡 중요: waitKey(1)이 있어야 창이 유지되고 갱신됩니다.
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"⚠️ 영상 처리 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ChickenDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 창 모두 닫기
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
