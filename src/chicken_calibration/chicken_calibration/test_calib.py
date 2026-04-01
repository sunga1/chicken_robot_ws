#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from dsr_msgs2.srv import MoveLine, MoveStop

class ChickenChaser(Node):
    def __init__(self):
        super().__init__('chicken_chaser_node')
        
        # 1. 캘리브레이션 결과 (m 단위)
        self.t_c2g = np.array([-0.04861006, 0.05671699, 0.16817704]) 
        self.q_c2g = [0.06996341, 0.01519766, 0.97633869, -0.20405127] 
        self.R_c2g = R.from_quat(self.q_c2g).as_matrix()

        # 그리퍼 오프셋 (230mm)
        self.gripper_offset = 230.0

        # 2. ROS 2 TF 및 서비스 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_cli = self.create_client(MoveLine, '/motion/move_line')
        self.stop_cli = self.create_client(MoveStop, '/motion/move_stop')

        # 3. 리얼센스 카메라 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        # 4. 노란색 감지 범위
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])

        self.create_timer(0.1, self.main_loop)
        self.get_logger().info("Node Started. [m]: Move XY, [o]: Move Z+10, [u]: Up 10cm, [s]: Stop, [q]: Quit")

    def main_loop(self):
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame: return

            img = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # 1. 현재 로봇 위치는 공 인식 여부와 상관없이 매 루프마다 가져옵니다.
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'link_6', now, rclpy.duration.Duration(seconds=0.1))
            cur_t = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            robot_z_mm = cur_t[2] * 1000.0

            # 2. 공 인식 로직
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            b_point = None # 공의 위치 초기화

            if contours:
                c = max(contours, key=cv2.contourArea)
                ((u_pix, v_pix), radius) = cv2.minEnclosingCircle(c)
                
                if radius > 15:
                    u_pix, v_pix = int(u_pix), int(v_pix)
                    dist = depth_frame.get_distance(u_pix, v_pix)
                    
                    if dist > 0:
                        c_point = rs.rs2_deproject_pixel_to_point(self.intr, [u_pix, v_pix], dist)
                        cur_R = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, 
                                            trans.transform.rotation.z, trans.transform.rotation.w]).as_matrix()

                        g_point = self.R_c2g @ np.array(c_point) + self.t_c2g
                        b_point = cur_R @ g_point + cur_t
                        
                        ball_z_mm = b_point[2] * 1000.0
                        target_pick_z = ball_z_mm + self.gripper_offset + 10.0
                        z_diff = robot_z_mm - target_pick_z

                        cv2.circle(img, (u_pix, v_pix), int(radius), (0, 255, 0), 2)
                        cv2.putText(img, f"Z-Diff to Pick: {z_diff:.1f} mm", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 3. [중요] 키 입력 처리를 공 인식 블록(if contours) 밖으로 이동
            key = cv2.waitKey(1)
            
            # (m)은 공이 인식되었을 때만 동작해야 함
            if key == ord('m') and b_point is not None:
                target_mm = [b_point[0]*1000.0, b_point[1]*1000.0, robot_z_mm]
                self.send_move_command(target_mm)
                self.get_logger().info(f"Horizontal Move: {target_mm[:2]}")

            # (o)는 공이 인식되었을 때만 동작해야 함
            elif key == ord('o') and b_point is not None:
                target_mm = [cur_t[0]*1000.0, cur_t[1]*1000.0, target_pick_z]
                self.send_move_command(target_mm)
                self.get_logger().info(f"Vertical Down: {target_mm[2]}mm")

            # (u)는 공이 있든 없든 현재 높이 기준으로 동작 가능
            elif key == ord('u'):
                target_mm = [cur_t[0]*1000.0, cur_t[1]*1000.0, robot_z_mm + 100.0]
                self.send_move_command(target_mm)
                self.get_logger().info(f"Vertical Up: {target_mm[2]}mm")

            elif key == ord('s'):
                self.send_stop_command()
            elif key == ord('q'):
                rclpy.shutdown()

            cv2.imshow("Vision Tracking", img)
            cv2.waitKey(1)

        except Exception as e:
            pass

    def send_move_command(self, pos_mm):
        if not self.move_cli.wait_for_service(timeout_sec=1.0): return
        req = MoveLine.Request()
        # [X, Y, Z, rx, ry, rz] 수직 고정
        req.pos = [pos_mm[0], pos_mm[1], pos_mm[2], 0.0, 180.0, 0.0] 
        req.vel = [60.0, 60.0]
        req.acc = [60.0, 60.0]
        self.move_cli.call_async(req)

    def send_stop_command(self):
        if not self.stop_cli.wait_for_service(timeout_sec=1.0): return
        req = MoveStop.Request()
        req.stop_mode = 1
        self.stop_cli.call_async(req)
        self.get_logger().warn("Stop Command Sent")

def main():
    rclpy.init()
    node = ChickenChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
