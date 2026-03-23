#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveLine

class CameraToRobotMover(Node):
    def __init__(self):
        super().__init__('camera_mover_node')
        self.client = self.create_client(MoveLine, '/motion/move_line')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        # 1. 캘리브레이션 값 (로봇 베이스에서 카메라까지의 거리)
        self.cam_offset_x = 0.131
        self.cam_offset_y = -0.007
        self.cam_offset_z = 0.503

    def move_to_camera_coord(self, cam_x, cam_y, cam_z):
        # 2. 카메라 좌표를 로봇 좌표로 변환
        robot_x = self.cam_offset_x + cam_x
        robot_y = self.cam_offset_y - cam_y
        robot_z = self.cam_offset_z + cam_z

        # 3. mm 단위 변환 및 명령 생성
        req = MoveLine.Request()
        req.pos = [robot_x * 1000, robot_y * 1000, robot_z * 1000, 0.0, 180.0, 0.0]
        req.vel = [100.0, 100.0]
        req.acc = [100.0, 100.0]

        self.get_logger().info(f'로봇 기준 목표 좌표(mm): {req.pos}')
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init()
    mover = CameraToRobotMover()
    
    # 카메라가 알려준 좌표 입력
    mover.move_to_camera_coord(0.149, 0.00315, 0.504)
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
