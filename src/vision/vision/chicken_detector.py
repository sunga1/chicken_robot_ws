import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from ultralytics import YOLO

class ChickenDetector(Node):
    def __init__(self):
        super().__init__('chicken_detector')
        self.bridge = CvBridge()
        
        # 1. 모델 경로 설정
        model_path = os.path.join(os.path.expanduser('~'), 'Downloads', 'best.pt')
        self.model = YOLO(model_path)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. 퍼블리셔 설정
        self.pose_pub = self.create_publisher(PoseArray, '/detected_chickens', 10)
        self.image_pub = self.create_publisher(Image, '/detected_chicken_image', 10)
        
        # 3. 서브스크라이버 설정
        self.cmd_sub = self.create_subscription(String, '/robot/command', self.cmd_cb, 10)
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)

        self.latest_color = None
        self.latest_depth = None
        self.intrinsics = None 

        self.is_scanning = False
        self.create_timer(0.1, self.detection_loop)

    def info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = msg
            self.get_logger().info("카메라 내상수(Intrinsics) 정보를 수신했습니다.")

    def color_cb(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def cmd_cb(self, msg):
        if msg.data == "START_SCAN": 
            self.is_scanning = True
            self.get_logger().info("Vision: 스캔 명령 수신 - 탐지를 시작합니다.")

    def deproject_pixel_to_point(self, u, v, depth):
        """픽셀 좌표와 뎁스를 카메라 3D 좌표(m)로 변환"""
        z = float(depth) * 0.001 
        x = (u - self.intrinsics.k[2]) * z / self.intrinsics.k[0]
        y = (v - self.intrinsics.k[5]) * z / self.intrinsics.k[4]
        return np.array([x, y, z])

    def get_mat(self, t):
        """TF 변환을 행렬로 변환"""
        pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        rot = R.from_quat([
            t.transform.rotation.x, t.transform.rotation.y, 
            t.transform.rotation.z, t.transform.rotation.w
        ]).as_matrix()
        return pos, rot

    def detection_loop(self):
        # 기본 데이터 체크
        if self.latest_color is None or self.latest_depth is None or self.intrinsics is None:
            return

        try:
            debug_img = self.latest_color.copy()
            depth_img = self.latest_depth

            # 1. YOLO 추론 (무조건 실행하여 rqt 확인 가능하게 함)
            results = self.model.predict(debug_img, conf=0.5, verbose=False)
            
            # 2. 결과 시각화 로직
            for r in results:
                if r.boxes is not None:
                    for box in r.boxes:
                        b = box.xyxy[0].cpu().numpy()
                        cv2.rectangle(debug_img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
                
                if r.masks is not None:
                    for mask_data in r.masks.xy:
                        mask_pts = mask_data.astype(np.int32)
                        cv2.polylines(debug_img, [mask_pts], True, (0, 0, 255), 2)

            # 시각화 영상 발행
            img_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.image_pub.publish(img_msg)

            # 3. 데이터 전송 로직 (is_scanning 일 때만)
            if self.is_scanning:
                # TF 정보 가져오기
                try:
                    now = rclpy.time.Time()
                    t_b2l6 = self.tf_buffer.lookup_transform('base_link', 'link_6', now, rclpy.duration.Duration(seconds=0.1))
                    t_l62c = self.tf_buffer.lookup_transform('link_6', 'camera_link', now, rclpy.duration.Duration(seconds=0.1))
                    p_b2l6, r_b2l6 = self.get_mat(t_b2l6)
                    p_l62c, r_l62c = self.get_mat(t_l62c)
                except Exception as e:
                    self.get_logger().warn(f"TF Wait: {e}")
                    return

                # --- [중요] 모든 치킨 정보를 담을 바구니를 루프 밖에서 생성 ---
                msg_out = PoseArray()
                msg_out.header.frame_id = "base_link"
                msg_out.header.stamp = self.get_clock().now().to_msg()

                for r in results:
                    if r.masks is None: continue
                    
                    # 각 마스크마다 반복 (하나의 results 안에 여러 마스크가 있을 수 있음)
                    for mask_data in r.masks.xy:
                        mask_pts = mask_data.astype(np.int32)
                        
                        # 중심점 계산
                        M = cv2.moments(mask_pts)
                        if M['m00'] == 0: continue
                        u, v = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                        
                        # 뎁스 확인
                        dist_raw = depth_img[v, u]
                        if dist_raw == 0: 
                            self.get_logger().debug("중심점의 Depth 값이 0이라 건너뜁니다.")
                            continue 

                        # 좌표 변환 (카메라 -> 베이스 링크)
                        c_p = self.deproject_pixel_to_point(u, v, dist_raw)
                        b_p = r_b2l6 @ (r_l62c @ c_p + p_l62c) + p_b2l6

                        # PCA 각도 계산 (주황색 선의 방향)
                        _, eig = cv2.PCACompute(mask_pts.astype(np.float32), mean=None)
                        angle = np.degrees(np.arctan2(eig[0][1], eig[0][0])) + 90.0

                        # Pose 생성 및 리스트에 추가 (append)
                        p = Pose()
                        p.position.x = float(b_p[0])
                        p.position.y = float(b_p[1])
                        p.position.z = float(b_p[2])
                        p.orientation.z = float(angle) 
                        msg_out.poses.append(p) # <--- 찾은 만큼 다 담습니다.

                # 4. 모든 치킨 탐지가 끝난 후 한 번에 발행
                if msg_out.poses:
                    self.pose_pub.publish(msg_out)
                    self.get_logger().info("-" * 40)
                    self.get_logger().info(f"Vision: 총 {len(msg_out.poses)}개의 치킨 좌표를 Planner에게 전송!")
                    self.get_logger().info("-" * 40)
                    self.is_scanning = False # 전송 후 대기 상태로 전환
                else:
                    self.get_logger().warn("치킨은 보이지만 유효한 좌표(Depth 등)를 찾지 못했습니다.")

        except Exception as e:
            self.get_logger().error(f"Detection Loop 에러: {e}")

def main():
    rclpy.init()
    node = ChickenDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
