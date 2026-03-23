#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster

class CalibrationVisualizer(Node):
    def __init__(self):
        super().__init__('cal_viz_node')
        
        # 1. 캘리브레이션 TF 발행 (로봇 베이스 -> 카메라)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_calibration_tf()

        # 2. RViz에 점 찍기 (카메라 -> 물체)
        self.point_publisher = self.create_publisher(PointStamped, '/detected_object_point', 10)
        self.timer = self.create_timer(0.5, self.publish_point)
        
        self.get_logger().info('캘리브레이션 데이터 적용 및 시각화 시작')

    def publish_calibration_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        # 우리가 계산한 카메라 설치 위치 (m 단위)
        t.transform.translation.x = 0.131
        t.transform.translation.y = -0.007
        t.transform.translation.z = 0.503

        # 회전값 (일단 정면을 보도록 기본값 설정, 필요시 RPY 수정)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def publish_point(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 카메라 렌즈(optical_frame) 기준으로 점을 찍습니다.
        msg.header.frame_id = 'camera_link' 

        # 카메라가 알려준 물체 좌표 그대로 입력
        msg.point.x = -1.1
        msg.point.y = -0.221
        msg.point.z = 0.14
        
        self.point_publisher.publish(msg)

def main():
    rclpy.init()
    node = CalibrationVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
