import rclpy
from rclpy.node import Node
import numpy as np
import time # 추가: sleep 사용을 위해 필요
from scipy.spatial.transform import Rotation as R
# from pymodbus.client import ModbusTcpClient # 주석 처리

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from dsr_msgs2.srv import MoveLine, MoveStop

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # 1. Gripper 설정 제외 (테스트용)
        self.get_logger().info("Gripper Modbus Bypass Mode (하드웨어 없이 테스트)")
        self.modbus_client = None # 클라이언트 객체만 생성 방지
        
        # 2. 파라미터 및 변환 행렬
        self.gripper_offset = 230.0  # mm
        self.fixed_angle = 0.0

        # 3. ROS 서비스 클라이언트
        self.move_cli = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.stop_cli = self.create_client(MoveStop, '/dsr01/motion/move_stop')

        # 4. 통신 설정
        self.status_pub = self.create_publisher(Bool, '/robot/status_done', 10)
        self.cmd_sub = self.create_subscription(String, '/robot/command', self.command_callback, 10)
        self.target_sub = self.create_subscription(Pose, '/robot/target_pose', self.pick_callback, 10)

        # 상태 관리
        self.is_moving = False

    # --- 그리퍼 제어 함수 (로그만 남기고 동작 제외) ---
    def set_gripper_modbus(self, mode):
        # 실제 장비가 없으므로 로그만 출력하고 바로 반환합니다.
        self.get_logger().info(f"[가상] Gripper {mode} 명령 수행 완료")

    # --- 이동 명령 실행 함수 ---
    def send_move_command(self, pos_mm, rz_deg):
        if not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Doosan Robotics Service (move_line)를 찾을 수 없습니다!")
            return None
        
        req = MoveLine.Request()
        # [x, y, z, a, b, c]
        req.pos = [float(pos_mm[0]), float(pos_mm[1]), float(pos_mm[2]), 0.0, 180.0, float(rz_deg)]
        req.vel = [60.0, 60.0]
        req.acc = [60.0, 60.0]
        
        self.get_logger().info(f"이동 명령 전송: {req.pos}")
        return self.move_cli.call_async(req)

    # --- Task Planner 명령 처리 ---
    def command_callback(self, msg):
        if msg.data == "GO_HOME":
            self.get_logger().info("시야 확보 위치(Home)로 이동 시퀀스")
            self.send_move_command([-714.83, 2.21, 1218.85], 0.0)
            # 실제로는 이동이 끝난 후 report_done을 해야 하지만, 테스트를 위해 2초 후 완료 보고
            time.sleep(2.0) 
            self.report_done()

    def pick_callback(self, msg):
        """Task Planner가 보낸 '가장 높은 치킨' 좌표로 시퀀스 실행"""
        if self.is_moving: return
        self.is_moving = True

        # 1. m -> mm 변환
        target_x = msg.position.x * 1000.0
        target_y = msg.position.y * 1000.0
        target_z = msg.position.z * 1000.0
        target_rz = msg.orientation.z 

        # 2. 오프셋 적용
        pick_z = target_z + self.gripper_offset + 30.0
        approach_z = pick_z + 100.0 

        self.get_logger().info(f"📍 가상 픽업 시작: Target Z={target_z:.1f} -> Pick Z={pick_z:.1f}")

        # 시퀀스 (그리퍼 동작은 로그만 남음)
        self.set_gripper_modbus("OPEN")
        self.send_move_command([target_x, target_y, approach_z], target_rz)
        time.sleep(2.0) 
        
        self.send_move_command([target_x, target_y, pick_z], target_rz)
        time.sleep(2.0)
        
        self.set_gripper_modbus("CLOSE")
        time.sleep(1.5)
        
        self.send_move_command([target_x, target_y, approach_z], target_rz)
        time.sleep(2.0)
        
        self.is_moving = False
        self.report_done()
        self.get_logger().info("✅ 픽업 시퀀스 종료")

    def report_done(self):
        msg = Bool()
        msg.data = True
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.modbus_client.close() # 주석 처리
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
