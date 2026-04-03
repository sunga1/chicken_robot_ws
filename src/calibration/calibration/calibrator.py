#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import threading
import os
import yaml
import time
from dsr_msgs2.srv import MoveLine
import sys, select, tty, termios
from tf2_ros import Buffer, TransformListener

class RealSenseCalibrator(Node):
    def __init__(self):
        super().__init__('rs_eye_in_hand_calib')
        
        # 1. 체커보드 설정 (실측 6x9 점)
        self.checkerboard_dims = (6, 9) 
        self.square_size = 0.03 # 30mm
        
        # 2. 이동 좌표 리스트 [x, y, z, rx, ry, rz]
        self.test_points = [ 
            [-519.51, -41.03, 1271.77, 12.21, -165.76, -83.57],
            [-475.89, -96.56, 1213.28, 142.01, 159.51, 37.89],
            [-371.31, -27.67, 1286.70, 147.46, 154.91, 51.11],
            [-767.34, -114.73, 1235.42, 38.25, 163.24, -59.51],
            [-745.51, 3.19, 1279.92, 23.08, 168.15, -66.61],
            [-659.41, 226.41, 1298.11, 98.30, -158.50, 21.30],
            [-560.84, 212.84, 1371.51, 79.56, -161.06, 3.98],
            [-520.48, 351.02, 1313.42, 72.34, -147.79, 7.54],
            [-602.59, 146.76, 1312.56, 91.33, -164.37, 10.98],
            [-660.06, -50.28, 1283.76, 171.40, -173.31, 75.71],
            [-591.60, 82.13, 1256.61, 41.49, -172.16, -50.60],
            [-419.24, 217.12, 1229.82, 46.37, -159.13, -25.76],
            [-459.39, -68.72, 1125.69, 149.37, 157.00, 36.99],
            [-358.83, -194.46, 1237.05, 146.53, 145.63, 32.03],
            [-381.71, -148.62, 1369.81, 169.74, 156.01, 134.23],
            [-444.60, -141.43, 1226.26, 166.50, 155.74, 131.51],
            [-328.08, -423.92, 1220.37, 144.52, 142.43, 77.21]
        ]
        self.current_pt_idx = 0

        # 3. 로봇 데이터 및 통신 설정
        self.target_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 서비스 클라이언트 생성
        self.move_cli = self.create_client(MoveLine, '/motion/move_line')
        
        self.R_gripper2base, self.t_gripper2base = [], []
        self.R_target2cam, self.t_target2cam = [], []
        
        # 4. 리얼센스 초기화
        self.init_realsense()

        # 5. 스레드 시작
        self.stop_threads = False
        self.img_thread = threading.Thread(target=self.image_loop, daemon=True)
        self.img_thread.start()
        
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("RealSense Hand-Eye Calibration 시작")
        self.get_logger().info("   'n': 다음 포인트로 이동 명령 전송")
        self.get_logger().info("   's': 현재 위치에서 샘플 저장 (격자 확인 후)")
        self.get_logger().info("   'q': 데이터 계산, YAML 저장 및 종료")
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
        try:
            while rclpy.ok() and not self.stop_threads:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame: continue

                img = np.asanyarray(color_frame.get_data())
                display_img = img.copy()
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

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
        if self.current_pt_idx >= len(self.test_points):
            self.get_logger().warn("모든 테스트 포인트를 순회했습니다.")
            return

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
        self.get_logger().info(f"[{self.current_pt_idx + 1}/{len(self.test_points)}] 로봇 이동 중...")
        self.current_pt_idx += 1

    def save_sample(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'link_6', now, rclpy.duration.Duration(seconds=1.0))
            
            frames = self.pipeline.wait_for_frames()
            img = np.asanyarray(frames.get_color_frame().get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_dims, cv2.CALIB_CB_ADAPTIVE_THRESH)

            if ret:
                objp = np.zeros((self.checkerboard_dims[0]*self.checkerboard_dims[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:self.checkerboard_dims[0], 0:self.checkerboard_dims[1]].T.reshape(-1, 2) * self.square_size
                _, rvec, tvec = cv2.solvePnP(objp, corners, self.mtx, self.dist)
                R_tc, _ = cv2.Rodrigues(rvec)

                r_robot = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, 
                                       trans.transform.rotation.z, trans.transform.rotation.w])
                
                self.R_gripper2base.append(r_robot.as_matrix())
                self.t_gripper2base.append([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                self.R_target2cam.append(R_tc)
                self.t_target2cam.append(tvec)
                
                self.get_logger().info(f"샘플 {len(self.R_gripper2base)} 저장 성공")
            else:
                self.get_logger().error("체커보드 인식 실패")
        except Exception as e:
            self.get_logger().error(f"데이터 저장 오류: {e}")

    def compute_calibration(self):
        if len(self.R_gripper2base) < 5:
            self.get_logger().error("데이터 부족(최소 5개 필요)")
            return
        
        R_g2b = np.array(self.R_gripper2base, dtype=np.float64)
        t_g2b = np.array(self.t_gripper2base, dtype=np.float64)
        R_t2c = np.array(self.R_target2cam, dtype=np.float64)
        t_t2c = np.array(self.t_target2cam, dtype=np.float64)

        # 캘리브레이션 실행
        R_c2g, t_c2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c)
        quat = R.from_matrix(R_c2g).as_quat()

        # YAML 저장 경로 설정
        workspace_path = os.path.expanduser('~/chicken_robot_ws')
        config_dir = os.path.join(workspace_path, 'src/chicken_calibration/config')
        
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
            
        file_path = os.path.join(config_dir, 'calibration_val.yaml')

        calib_dict = {
            'calibration_data': {
                'translation': {
                    'x': float(t_c2g[0]),
                    'y': float(t_c2g[1]),
                    'z': float(t_c2g[2])
                },
                'quaternion': {
                    'x': float(quat[0]),
                    'y': float(quat[1]),
                    'z': float(quat[2]),
                    'w': float(quat[3])
                },
                'frame_id': 'link_6',
                'child_frame_id': 'camera_link',
                'description': 'Eye-in-Hand Calibration Result',
                'date': time.strftime("%Y-%m-%d %H:%M:%S")
            }
        }

        with open(file_path, 'w') as f:
            yaml.dump(calib_dict, f, default_flow_style=False)

        self.get_logger().info("\n" + "="*50)
        self.get_logger().info(f"캘리브레이션 완료 및 파일 저장 완료")
        self.get_logger().info(f"경로: {file_path}")
        self.get_logger().info(f"T (m): {t_c2g.flatten()}")
        self.get_logger().info(f"Q (xyzw): {quat}")
        self.get_logger().info("="*50)

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
                    elif key == 'q': 
                        self.compute_calibration()
                        self.stop_threads = True
                        break
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
