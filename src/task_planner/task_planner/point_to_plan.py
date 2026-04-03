#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
import math

class PointToPlanNode(Node):
    def __init__(self):
        super().__init__('point_to_plan_node')
        
        # 1. 캘리브레이션 오프셋 (확정된 값)
        self.off_x, self.off_y, self.off_z = 0.131, -0.007, 0.503

        # 2. MoveIt Planning 서비스 클라이언트
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # 3. Publish Point 구독
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.listener_callback,
            10)
        
        self.get_logger().info('=== Plan 전용 노드 실행 중 (Group: manipulator) ===')

    def listener_callback(self, msg):
        # 카메라 좌표 -> 로봇 좌표 변환
        r_x = self.off_x + msg.point.x
        r_y = self.off_y - msg.point.y
        r_z = self.off_z + msg.point.z

        self.get_logger().info(f'목표 좌표 수신: x={r_x:.3f}, y={r_y:.3f}, z={r_z:.3f}')
        self.request_moveit_plan(r_x, r_y, r_z)

    def request_moveit_plan(self, x, y, z):
        if not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('MoveIt 서비스를 찾을 수 없습니다.')
            return

        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'manipulator' # 확인된 그룹명
        req.motion_plan_request.num_planning_attempts = 5
        req.motion_plan_request.allowed_planning_time = 2.0
        
        # 시작 상태 에러 방지
        req.motion_plan_request.start_state.is_diff = True

        # [핵심 수정] Goal Constraints 설정
        goal_constraints = Constraints()
        
        # 1. 위치 제약 (Position Constraint)
        pos_con = PositionConstraint()
        pos_con.header.frame_id = 'base_link'
        pos_con.link_name = 'link6' # 로봇 끝단 링크 이름 (h2017의 경우 확인 필요, 보통 link6)
        pos_con.constraint_region.primitive_poses.append(PoseStamped().pose)
        pos_con.constraint_region.primitive_poses[0].position.x = x
        pos_con.constraint_region.primitive_poses[0].position.y = y
        pos_con.constraint_region.primitive_poses[0].position.z = z
        
        # 허용 오차 (1cm)
        from shape_msgs.msg import SolidPrimitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        pos_con.constraint_region.primitives.append(box)
        pos_con.weight = 1.0

        # 2. 자세 제약 (Orientation Constraint - 바닥 바라보기)
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = 'base_link'
        ori_con.link_name = 'link_6'
        ori_con.orientation.x = 0.0
        ori_con.orientation.y = 1.0
        ori_con.orientation.z = 0.0
        ori_con.orientation.w = 0.0
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0

        goal_constraints.position_constraints.append(pos_con)
        goal_constraints.orientation_constraints.append(ori_con)
        req.motion_plan_request.goal_constraints.append(goal_constraints)

        self.get_logger().info('MoveIt에 경로 계획을 요청했습니다.')
        self.plan_client.call_async(req)

def main():
    rclpy.init()
    node = PointToPlanNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
