#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from dsr_msgs2.srv import MoveLine, MoveStop, SetCtrlIO # IO 서비스 추가
from ultralytics import YOLO

class ChickenChaser(Node):
    def __init__(self):
        super().__init__('chicken_chaser_node')
        
        # 1. 설정 및 캘리브레이션
        self.t_c2g = np.array([-0.04861006, 0.05671699, 0.16817704]) 
        self.q_c2g = [0.06996341, 0.01519766, 0.97633869, -0.20405127] 
        self.R_c2g = R.from_quat(self.q_c2g).as_matrix()
        self.gripper_offset = 230.0

        # 2. 모델 로드
        home_dir = os.path.expanduser('~')
        model_path = os.path.join(home_dir, 'Downloads', 'best.pt')
        self.model = YOLO(model_path)

        # 3. 제어 변수
        self.angle_buffer = [] 
        self.fixed_angle = 0.0
        self.gripper_pin = 1 # 온로봇 그리퍼가 연결된 IO 핀 번호 (확인 필요)
        
        # ROS 서비스 클라이언트
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_cli = self.create_client(MoveLine, '/motion/move_line')
        self.stop_cli = self.create_client(MoveStop, '/motion/move_stop')
        self.io_cli = self.create_client(SetCtrlIO, '/io/set_digital_output') # IO 클라이언트

        # 카메라 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.create_timer(0.1, self.main_loop)
        self.get_logger().info("OnRobot Ready. [g]: Open, [h]: Close, [m]: Aim, [o]: Pick")

    def set_gripper(self, state):
        """그리퍼 열기(0) / 닫기(1) 제어"""
        if not self.io_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("IO 서비스를 찾을 수 없습니다!")
            return
        req = SetCtrlIO.Request()
        req.port = self.gripper_pin
        req.value = 1 if state else 0 # 1은 닫기(ON), 0은 열기(OFF)
        self.io_cli.call_async(req)
        self.get_logger().info(f"Gripper {'Close' if state else 'Open'} 전송")

    def get_stable_angle(self, mask_coords):
        rect = cv2.minAreaRect(mask_coords)
        raw_angle = rect[2]
        width, height = rect[1]
        # 장축 기준 정규화 및 90도 보정
        angle = raw_angle + (180.0 if width < height else 90.0) + 90.0
        
        self.angle_buffer.append(angle)
        if len(self.angle_buffer) > 10: self.angle_buffer.pop(0)
        return np.mean(self.angle_buffer)

    def main_loop(self):
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame: return

            img = np.asanyarray(color_frame.get_data())
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'link_6', now, rclpy.duration.Duration(seconds=0.1))
            cur_t = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            robot_z_mm = cur_t[2] * 1000.0

            results = self.model.predict(img, conf=0.5, verbose=False)
            b_point, current_view_angle, target_pick_z = None, 0.0, 0.0

            for r in results:
                if r.masks is not None and len(r.masks.xy) > 0:
                    mask_coords = r.masks.xy[0].astype(np.int32)
                    cv2.polylines(img, [mask_coords], True, (0, 255, 0), 2)
                    current_view_angle = self.get_stable_angle(mask_coords)
                    
                    M = cv2.moments(mask_coords)
                    if M['m00'] != 0:
                        u_pix, v_pix = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                        dist = depth_frame.get_distance(u_pix, v_pix)
                        if dist > 0:
                            c_point = rs.rs2_deproject_pixel_to_point(self.intr, [u_pix, v_pix], dist)
                            cur_R = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, 
                                                trans.transform.rotation.z, trans.transform.rotation.w]).as_matrix()
                            g_point = self.R_c2g @ np.array(c_point) + self.t_c2g
                            b_point = cur_R @ g_point + cur_t
                            b_point[1] += 0.02
                            target_pick_z = (b_point[2] * 1000.0) + self.gripper_offset + 10.0

                            cv2.circle(img, (u_pix, v_pix), 5, (0, 0, 255), -1)
                            cv2.putText(img, f"FIXED: {self.fixed_angle:.1f}", (10, 60), 0, 0.7, (0, 255, 255), 2)

            key = cv2.waitKey(1) & 0xFF
            # --- 그리퍼 수동 제어 ---
            if key == ord('g'): self.set_gripper(False) # Open
            elif key == ord('h'): self.set_gripper(True) # Close
            
            # --- 로봇 이동 제어 ---
            elif key == ord('m') and b_point is not None:
                self.fixed_angle = current_view_angle
                self.send_move_command([b_point[0]*1000.0, b_point[1]*1000.0, robot_z_mm], self.fixed_angle)
            elif key == ord('o') and b_point is not None:
                self.send_move_command([cur_t[0]*1000.0, cur_t[1]*1000.0, target_pick_z], self.fixed_angle)
            elif key == ord('u'):
                self.send_move_command([cur_t[0]*1000.0, cur_t[1]*1000.0, robot_z_mm + 100.0], 0.0)
            elif key == ord('s'): self.send_stop_command()
            elif key == ord('q'): rclpy.shutdown()

            cv2.imshow("Stable Chicken Chaser & OnRobot", img)
        except Exception: pass

    def send_move_command(self, pos_mm, rz_deg):
        if not self.move_cli.wait_for_service(timeout_sec=1.0): return
        req = MoveLine.Request()
        req.pos = [pos_mm[0], pos_mm[1], pos_mm[2], 0.0, 180.0, float(rz_deg)] 
        req.vel, req.acc = [60.0, 60.0], [60.0, 60.0]
        self.move_cli.call_async(req)

    def send_stop_command(self):
        if not self.stop_cli.wait_for_service(timeout_sec=1.0): return
        req = MoveStop.Request()
        req.stop_mode = 1
        self.stop_cli.call_async(req)

def main():
    rclpy.init()
    node = ChickenChaser()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
