import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseArray, Pose

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')

        # 1. 상태 정의 (IDLE -> MOVING_HOME -> SCANNING -> PICKING -> FINISHED)
        self.state = "IDLE"
        self.target_chicken = None

        # 2. Publisher: Manipulation에게 명령 전달
        self.cmd_pub = self.create_publisher(String, '/robot/command', 10)
        self.target_pub = self.create_publisher(Pose, '/robot/target_pose', 10)

        # 3. Subscriber: 로봇 상태 및 비전 결과 수신
        self.status_sub = self.create_subscription(Bool, '/robot/status_done', self.status_callback, 10)
        self.vision_sub = self.create_subscription(PoseArray, '/detected_chickens', self.vision_callback, 10)


        self.start_timer = self.create_timer(1.0, self.initiate_process)
	
    def initiate_process(self):
        self.start_timer.cancel() # 타이머 한 번만 실행하고 종료
        self.get_logger().info("--- Chicken Chaser 프로세스 시작 ---")
        self.send_command("GO_HOME")
        self.state = "MOVING_HOME"

    def send_command(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"명령 전송: {cmd_str}")

    def status_callback(self, msg):
        """Manipulation 노드가 이동을 완료했을 때 호출됨"""
        if msg.data is True:
            if self.state == "MOVING_HOME":
                self.get_logger().info("홈 위치 도착 완료! 1초 후 인식을 시작합니다.")
                self.state = "SCANNING"
            
            elif self.state == "PICKING":
                self.get_logger().info("치킨 픽업 완료! 미션을 종료합니다.")
                self.state = "FINISHED"

    def vision_callback(self, msg):
        """Vision 노드에서 치킨 리스트를 보내줄 때 호출됨"""
        # 스캐닝 상태일 때만 데이터를 처리합니다.
        if self.state != "SCANNING":
            return

        if not msg.poses:
            self.get_logger().warn("화면에 인식된 치킨이 없습니다...")
            return

        # --- [핵심 로직] 가장 높은(Z값이 작은) 치킨 찾기 ---
        # 카메라 좌표계에서 Z(Depth)가 작을수록 카메라와 가깝습니다.
        selected_chicken = min(msg.poses, key=lambda p: p.position.z)
        
        self.get_logger().info(f"최적 타겟 발견! Depth: {selected_chicken.position.z:.3f}m")
        
        # 선택된 좌표를 Manipulation에게 전송 (이것이 pick_up 명령이 됨)
        self.target_pub.publish(selected_chicken)
        self.state = "PICKING"
        self.get_logger().info("Manipulation에게 픽업 좌표를 전송했습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
