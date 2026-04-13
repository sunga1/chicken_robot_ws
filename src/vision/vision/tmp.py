import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YOLOCompare(Node):
    def __init__(self):
        super().__init__('yolo_compare')

        self.bridge = CvBridge()

        # 모델 로드
        self.model = YOLO('/home/sunga/chicken_robot_ws/src/vision/models/best.pt')

        # 토픽 구독 (너 환경 기준)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.callback,
            10
        )

    def draw_masks(self, frame, result):
        """
        세그멘테이션 마스크를 윤곽선(선)으로만 그리는 함수
        """
        if result.masks is None:
            return frame

        masks = result.masks.xy  # polygon 좌표

        for i, mask in enumerate(masks):
            # confidence 필터 (원하면 조절)
            if result.boxes is not None:
                conf = float(result.boxes.conf[i])
                if conf < 0.5:
                    continue

            pts = np.array(mask, dtype=np.int32)

            # 윤곽선 그리기 (초록색)
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        return frame

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            print("cv_bridge error:", e)
            return

        # YOLO 추론
        result = self.model(
    		frame,
    		imgsz=960,
    		conf=0.3,
    		iou=0.5
		)[0]

        # 원본 복사
        img = frame.copy()

        # 윤곽선만 그리기
        img = self.draw_masks(img, result)

        # 화면 출력
        cv2.imshow("Segmentation (mask only)", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YOLOCompare()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
