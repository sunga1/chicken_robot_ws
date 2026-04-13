import rclpy
from rclpy.node import Node
import numpy as np
import time

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from dsr_msgs2.srv import MoveLine


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.get_logger().info("🔥 ARM CONTROLLER STARTED")
        # 서비스 클라이언트
        self.move_cli = self.create_client(MoveLine, '/motion/move_line')

        # 상태 publish (Planner에 완료 알림)
        self.status_pub = self.create_publisher(Bool, '/robot/status_done', 10)

        # target pose 받기
        self.target_sub = self.create_subscription(
            Pose,
            '/robot/target_pose',
            self.pick_callback,
            10
        )

        self.is_moving = False

        # 🔥 파라미터
        self.gripper_offset = 230.0  # mm
        self.approach_offset = 100.0  # mm

    # 🔥 이동 명령
    def send_move(self, x, y, z, rz):
        if not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("move_line 서비스 없음")
            return

        req = MoveLine.Request()
        req.pos = [x, y, z, 0.0, 180.0, rz]
        req.vel = [60.0, 60.0]
        req.acc = [60.0, 60.0]

        self.get_logger().info(f"이동: {req.pos}")
        self.move_cli.call_async(req)

    # 🔥 픽 시퀀스
    def pick_callback(self, msg):
        if self.is_moving:
            self.get_logger().warn("이미 이동 중")
            return

        self.is_moving = True

        # m → mm 변환
        x = msg.position.x * 1000.0
        y = msg.position.y * 1000.0
        z = msg.position.z * 1000.0
        rz = msg.orientation.z

        # 🔥 위치 계산
        pick_z = z + self.gripper_offset
        approach_z = pick_z + self.approach_offset

        self.get_logger().info(f"📍 타겟: ({x:.1f}, {y:.1f}, {z:.1f})")

        # 🔥 시퀀스 시작
        try:
            # 1. 접근
            self.send_move(x, y, approach_z, rz)
            time.sleep(2.0)

            # 2. 내려가기
            self.send_move(x, y, pick_z, rz)
            time.sleep(2.0)

            # 3. 그리퍼 닫기 (지금은 로그만)
            self.get_logger().info("🤏 Gripper CLOSE")
            time.sleep(1.0)

            # 4. 다시 올라오기
            self.send_move(x, y, approach_z, rz)
            time.sleep(2.0)

            self.get_logger().info("✅ 픽 완료")

        except Exception as e:
            self.get_logger().error(f"에러: {e}")

        # 🔥 완료 알림
        done_msg = Bool()
        done_msg.data = True
        self.status_pub.publish(done_msg)

        self.is_moving = False


def main():
    rclpy.init()
    node = ArmController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
