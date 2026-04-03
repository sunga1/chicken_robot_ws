#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
import os

class CalibTFBr(Node):
    def __init__(self):
        super().__init__('calib_tf_broadcaster')
        self.br = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        config_path = os.path.expanduser('~/chicken_robot_ws/src/chicken_calibration/config/calibration_val.yaml')
        
        if not os.path.exists(config_path):
            self.get_logger().error(f"설정 파일을 찾을 수 없음: {config_path}")
            return

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
            config = data['calibration_data']

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = config['frame_id']      # link_6
        t.child_frame_id = config['child_frame_id'] # camera_link

        t.transform.translation.x = config['translation']['x']
        t.transform.translation.y = config['translation']['y']
        t.transform.translation.z = config['translation']['z']
        
        t.transform.rotation.x = config['quaternion']['x']
        t.transform.rotation.y = config['quaternion']['y']
        t.transform.rotation.z = config['quaternion']['z']
        t.transform.rotation.w = config['quaternion']['w']

        self.br.sendTransform(t)
        self.get_logger().info(f"TF 시스템에 좌표계 등록 완료: [{config['frame_id']} -> {config['child_frame_id']}]")

def main():
    rclpy.init()
    node = CalibTFBr()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
