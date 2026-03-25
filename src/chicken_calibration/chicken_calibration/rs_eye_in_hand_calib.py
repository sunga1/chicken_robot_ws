#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import threading
from dsr_msgs2.srv import MoveLine
import sys, select, tty, termios
from tf2_ros import Buffer, TransformListener

class RealSenseCalibrator(Node):
    def __init__(self):
        super().__init__('rs_eye_in_hand_calib')
        
        # 1. 체커보드 설정 (실측 6x9 점)
        self.checkerboard_dims = (6, 9) 
        self.square_size = 0.02 # 20mm
        
        # 2. 성아님의 이동 좌표 리스트 [x, y, z, qx, qy, qz, qw]
        self.test_points = [
            [-1017.48, -227.88, 1176.76, 82.99, 175.59, 67.19],
[-966.30, -227.46, 1201.95, 89.88, 175.73, 73.48],
[-899.20, -161.46, 1234.03, 96.14, 175.85, 82.84],
[-868.69, -124.79, 1211.79, 169.68, 166.93, 157.66],
[-726.48, -147.49, 1281.67, 177.70, 162.50, 162.56],
[-684.49, -277.57, 1208.33, 0.38, -167.84, -25.59],
[-914.64, -142.32, 1046.52, 59.07, 174.20, 46.09],
[-1009.81, -114.53, 1056.67, 19.95, 161.93, 10.19],
[-1026.25, -231.55, 1082.00, 29.04, 165.05, 13.15],
[-922.98, -300.00, 1209.56, 99.26, 161.83, 80.32],
[-884.76, -482.86, 1129.83, 107.68, 142.78, 100.00],
[-612.92, -136.78, 807.06, 164.47, 130.14, 129.23],
[-737.58, 172.45, 791.50, 30.30, -122.59, -66.23],
[-649.28, 107.18, 772.92, 34.34, -129.52, -72.51],
[-708.22, -303.27, 1001.03, 150.32, 141.37, 129.79],
[-1047.95, -122.91, 960.65, 1.19, 152.44, 12.26],
[-1104.13, -137.69, 977.27, 178.19, -155.42, -171.47],
[-1052.13, -333.69, 935.53, 42.27, 149.29, 43.94],
        ]
        self.current_pt_idx = 0

        # 3. 로봇 데이터 및 통신 설정
        self.target_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 서비스 클라이언트를 미리 생성해둡니다 (수정됨)
        self.move_cli = self.create_client(MoveLine, '/motion/move_line')
        
        self.R_gripper2base, self.t_gripper2base = [], []
        self.R_target2cam, self.t_target2cam = [], []
        
        # 4. 리얼센스 초기화
        self.init_realsense()

        # 5. 스레드 시작 (영상 루프 + 키보드 루프)
        self.stop_threads = False
        self.img_thread = threading.Thread(target=self.image_loop, daemon=True)
        self.img_thread.start()
        
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("🚀 RealSense Hand-Eye Calibration 시작")
        self.get_logger().info("   'n': 다음 포인트로 이동 명령 전송")
        self.get_logger().info("   's': 현재 위치에서 샘플 저장 (격자 확인 후)")
        self.get_logger().info("   'q': 데이터 계산 및 프로그램 종료")
        self.get_logger().info("="*50)

    def init_realsense(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.mtx = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.dist = np.array(intr.coeffs)

    def image_loop(self):
        """실시간 화면 출력 및 체커보드 선 그리기"""
        try:
            while rclpy.ok() and not self.stop_threads:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame: continue

                img = np.asanyarray(color_frame.get_data())
                display_img = img.copy()
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # 체커보드 인식 (화면 출력용)
                ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_dims, 
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
                
                if ret:
                    cv2.drawChessboardCorners(display_img, self.checkerboard_dims, corners, ret)
                
                cv2.imshow('Calibration Viewer', display_img)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
        finally:
            cv2.destroyAllWindows()
            self.pipeline.stop()

    def move_next(self):
        """수정된 부분: 모든 명령어가 함수 안으로 들여쓰기 됨"""
        if self.current_pt_idx >= len(self.test_points):
            self.get_logger().warn("모든 포인트를 방문했습니다!")
            return

        # 서비스 대기 및 호출 로직이 함수 안으로 들어옴
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 이동 서비스 대기 중...')

        p = self.test_points[self.current_pt_idx]
        
        req = MoveLine.Request()
        req.pos = [p[0], p[1], p[2], p[3], p[4], p[5]] 
        req.vel = [100.0, 100.0] 
        req.acc = [100.0, 100.0]
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0 
        req.blend_type = 0

        self.move_cli.call_async(req)
        self.get_logger().info(f"🚀 [{self.current_pt_idx + 1}] 로봇에게 직접 이동 명령 전달!")
        self.current_pt_idx += 1

    def save_sample(self):
        """'s'를 누르면 현재 로봇 위치(TF)와 카메라 체커보드 위치 저장"""
        try:
            # 1. TF에서 로봇 끝단 위치 가져오기
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'link_6', now, rclpy.duration.Duration(seconds=1.0))
            
            # 2. 카메라에서 체커보드 포즈 계산
            frames = self.pipeline.wait_for_frames()
            img = np.asanyarray(frames.get_color_frame().get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_dims, cv2.CALIB_CB_ADAPTIVE_THRESH)

            if ret:
                objp = np.zeros((self.checkerboard_dims[0]*self.checkerboard_dims[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:self.checkerboard_dims[0], 0:self.checkerboard_dims[1]].T.reshape(-1, 2) * self.square_size
                _, rvec, tvec = cv2.solvePnP(objp, corners, self.mtx, self.dist)
                R_tc, _ = cv2.Rodrigues(rvec)

                # 데이터 저장
                r_robot = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, 
                                       trans.transform.rotation.z, trans.transform.rotation.w])
                self.R_gripper2base.append(r_robot.as_matrix())
                self.t_gripper2base.append([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                self.R_target2cam.append(R_tc)
                self.t_target2cam.append(tvec)
                
                self.get_logger().info(f"📸 샘플 {len(self.R_gripper2base)} 저장 성공!")
            else:
                self.get_logger().error("❌ 체커보드 인식 실패! 화면에 선이 다 나오는지 확인하세요.")
        except Exception as e:
            self.get_logger().error(f"❌ 데이터 저장 중 오류 발생: {e}")

    def compute_calibration(self):
        if len(self.R_gripper2base) < 5:
            self.get_logger().error("데이터는 최소 5개 이상 필요합니다.")
            return
        
        R_g2b = np.array(self.R_gripper2base, dtype=np.float64)
        t_g2b = np.array(self.t_gripper2base, dtype=np.float64)
        R_t2c = np.array(self.R_target2cam, dtype=np.float64)
        t_t2c = np.array(self.t_target2cam, dtype=np.float64)

        # 3. OpenCV 캘리브레이션 함수 실행
        R_c2g, t_c2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c)
        
        print("\n" + "="*50)
        print("Hand-Eye Calibration 결과 (Camera to Gripper)")
        print(f"Translation (m): {t_c2g.flatten()}")
        print(f"Rotation (Quat): {R.from_matrix(R_c2g).as_quat()}")
        print("="*50)

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and not self.stop_threads:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'n': self.move_next()
                    elif key == 's': self.save_sample()
                    elif key == 'q': self.compute_calibration(); self.stop_threads = True; break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = RealSenseCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_threads = True
        rclpy.shutdown()

if __name__ == '__main__':
    main()
