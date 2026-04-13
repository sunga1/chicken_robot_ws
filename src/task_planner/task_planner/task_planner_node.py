import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose


class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')

        self.state = "SCANNING"

        self.target_pub = self.create_publisher(Pose, '/robot/target_pose', 10)

        self.vision_sub = self.create_subscription(
            PoseArray,
            '/detected_chickens',
            self.vision_callback,
            10
        )

        self.confirm_sub = self.create_subscription(
            Bool,
            '/robot/confirm',
            self.confirm_callback,
            10
        )

        self.chicken_list = []
        self.current_target = None

    def vision_callback(self, msg):
        if self.state != "SCANNING":
            return

        if not msg.poses:
            return

        # depth 기준 정렬
        self.chicken_list = sorted(msg.poses, key=lambda p: p.position.z)

        self.current_target = self.chicken_list[0]

        self.get_logger().info(
            f"타겟 선택됨 (확인 대기) Z={self.current_target.position.z:.3f}"
        )

        self.state = "WAIT_CONFIRM"

    def confirm_callback(self, msg):
        if self.state != "WAIT_CONFIRM":
            return

        if msg.data:
            self.get_logger().info("사용자 승인 → 이동")

            self.target_pub.publish(self.current_target)

            # 다음 치킨 준비
            self.chicken_list.pop(0)

            if self.chicken_list:
                self.current_target = self.chicken_list[0]
                self.state = "WAIT_CONFIRM"
            else: 
                self.get_logger().info("모든 치킨 처리 완료")
                self.state = "FINISHED"


def main():
    rclpy.init()
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
