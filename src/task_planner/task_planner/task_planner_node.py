import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseArray, Pose


class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        
        self.state = "INIT"
        
        self.arm_cmd_pub = self.create_publisher(String, '/robot/arm_command', 10)
        self.target_pub = self.create_publisher(Pose, '/robot/target_pose', 10)
        
        self.status_sub = self.create_subscription(
            Bool,
            '/robot/status_done',
            self.status_callback,
            10
        )
        
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
        
        self.timer = self.create_timer(1.0, self.start_sequence)
        
        self.get_logger().info("TaskPlanner 시작")
        
    
    def start_sequence(self):
        self.timer.cancel()
        
        self.get_logger().info("홈 위치로 이동 시작")
        
        msg = String()
        msg.data = "GO_HOME"
        self.arm_cmd_pub.publish(msg)
        
        self.state = "MOVING_HOME"
        
    def status_callback(self, msg):
        if not msg.data:
            return
            
        if self.state == "MOVING_HOME"
            self.get_logger().info("홈 도착 완료 -> 스캔 시작")
            
            
            self.state = "SCANNING"
            
        elif self.state == "MOVING":
            self.get_logger().info("이동완료 -> 다음 치킨 대기")
            self.state = "WAIT_CONFIRM"
            
    
    def vision_callback(self, msg):
        if self.state != "SCANNING":
            return
            
        if not msg.poses:
            return
            
        self.chicken_list = sorted(msg.poses, key=lambda p: p.position.z)
        
        self.current_target = self.chicken_list[0]
        
        self.get_logger().info(
            f"타겟 선택됨 Z={self.current_target.position.z:.3f}"
        )
        
        self.state = "WAIT_CONFIRM"
        
    
    def confirm_callback(self, msg):
        if not msg.data:
            return
            
        if self.state != "WAIT_CONFIRM":
            self.get_logger().warn(f"지금 상태에서 입력 무시: {self.state}")
            return
            
        if self.current_target is None:
            self.get_logger().warn("타겟 없음")
            return
            
        self.get_logger().info("이동 시작")
        
        self.target_pub.publish(self.current_target)
        self.state = "MOVING"
        
        self.chicken_list.pop(0)
        
        if self.chicken_list:
            self.current_target = self.chicken_list[0]
            
        else:
            self.current_target = None
            
    
    
def main():
    rclpy.init()
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
        
            
            
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
