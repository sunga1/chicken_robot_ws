#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import time

class AutoCalibrator(Node):
    def __init__(self):
        super().__init__('auto_calibrator')
        self.bridge = CvBridge()
        
        # 1. 성아님이 직접 지정하는 로봇 이동 포인트 리스트 [x, y, z]
        # 로봇 베이스(base_link) 기준으로 안전한 경로를 입력하세요.
        self.test_points = [
            [0.45, 0.0, 0.4],
            [0.45, 0.1, 0.35],
            [0.45, -0.1, 0.35],
            [0.5, 0.0, 0.45],
            [0.4, 0.1, 0.5],
            [0.4, -0.1, 0.5]
        ]

        # 데이터 저장용
        self.robot_data = []  # 로봇 TCP 실제 좌표
        self.camera_data = [] # 카메라가 인식한 체커보드 3D 좌표
        
        # 서비스 클라이언트 (MoveIt Plan 요청)
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # 카메라 좌표 구독 (이전에 만든 point_to_plan에서 쓴 /clicked_point와 같은 형식 권장)
        # 또는 리얼센스에서 직접 체커보드 좌표를 쏴주는 토픽을 구독하세요.
        self.cam_sub = self.create_subscription(PointStamped, '/detected_checkerboard_point', self.cam_callback, 10)
        
        self.last_cam_point = None
        self.get_logger().info("=== 자동 캘리브레이션 노드 시작 ===")

    def cam_callback(self, msg):
        # 카메라가 실시간으로 인식 중인 체커보드의 3D 좌표 저장
        self.last_cam_point = [msg.point.x, msg.point.y, msg.point.z]

    def request_move(self, x, y, z):
        """MoveIt에 경로 계획 요청 (실행까지 포함되어야 함)"""
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            return False
        
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'manipulator' # 그룹명 확인!
        req.motion_plan_request.start_state.is_diff = True
        
        # 목표 Pose 설정 (바닥 바라보는 자세 고정)
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.y = 1.0 # 수직 하향
        target_pose.pose.orientation.w = 0.0

        # ... (이전의 Goal Constraints 설정 코드 생략 - 동일하게 적용 필요) ...
        
        self.get_logger().info(f"이동 중: {x, y, z}")
        self.plan_client.call_async(req)
        return True

    def start_calibration(self):
        for i, pt in enumerate(self.test_points):
            self.get_logger().info(f"[{i+1}/{len(self.test_points)}] 포인트로 이동합니다.")
            
            # 1. 로봇 이동
            self.request_move(pt[0], pt[1], pt[2])
            
            # 2. 이동 및 안정화 대기 (로봇 속도에 따라 조절)
            time.sleep(5.0) 
            
            # 3. 카메라 데이터 확인 및 저장
            if self.last_cam_point is not None:
                self.robot_data.append(pt)
                self.camera_data.append(self.last_cam_point)
                self.get_logger().info(f"데이터 수집 성공! (Camera: {self.last_cam_point})")
                self.last_cam_point = None # 다음 지점을 위해 초기화
            else:
                self.get_logger().warn("카메라가 체커보드를 찾지 못했습니다. 건너뜁니다.")

        self.calculate_result()

    def calculate_result(self):
        if len(self.robot_data) < 3:
            self.get_logger().error("데이터가 부족하여 계산할 수 없습니다.")
            return

        R = np.array(self.robot_data)
        C = np.array(self.camera_data)
        
        # Offset = Robot - Camera
        diff = R - C
        mean_offset = np.mean(diff, axis=0)
        std_dev = np.std(diff, axis=0)

        self.get_logger().info("\n" + "="*30)
        self.get_logger().info(f"최종 오프셋 결과 (m):")
        self.get_logger().info(f"OFF_X: {mean_offset[0]:.4f}")
        self.get_logger().info(f"OFF_Y: {mean_offset[1]:.4f}")
        self.get_logger().info(f"OFF_Z: {mean_offset[2]:.4f}")
        self.get_logger().info(f"오차 표준편차: {np.linalg.norm(std_dev):.4f}")
        self.get_logger().info("="*30)

def main():
    rclpy.init()
    node = AutoCalibrator()
    # 별도 스레드나 타이머로 start_calibration을 호출해야 spin이 막히지 않습니다.
    node.start_calibration() 
    rclpy.shutdown()
