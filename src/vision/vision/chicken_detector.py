import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO

from geometry_msgs.msg import PoseArray, Pose
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R


class ChickenDetector(Node):
    def __init__(self):
        super().__init__('chicken_detector')

        self.bridge = CvBridge()
        self.model = YOLO('/home/sunga/Downloads/best.pt')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ONLY PUBLISH (Vision 역할)
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/detected_chickens',
            10
        )

        # Camera
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_cb,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_cb,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_cb,
            10
        )

        self.latest_color = None
        self.latest_depth = None
        self.intrinsics = None

    def color_cb(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        self.process()

    def info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = msg

    # pixel → 3D
    def deproject(self, u, v, depth):
        z = depth * 0.001
        fx = self.intrinsics.k[0]
        fy = self.intrinsics.k[4]
        cx = self.intrinsics.k[2]
        cy = self.intrinsics.k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return np.array([x, y, z])

    # TF 변환
    def transform_to_base(self, cam_point):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time()
            )

            trans = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])

            rot = R.from_quat([
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ]).as_matrix()

            return rot @ cam_point + trans

        except Exception as e:
            self.get_logger().warn(f"TF 실패: {e}")
            return None

    def process(self):
        if self.latest_color is None or self.latest_depth is None or self.intrinsics is None:
            return

        frame = self.latest_color.copy()
        depth_img = self.latest_depth

        results = self.model(frame, imgsz=480, conf=0.4)[0]

        debug_img = frame.copy()

        msg_out = PoseArray()
        msg_out.header.frame_id = "base_link"
        msg_out.header.stamp = self.get_clock().now().to_msg()

        if results.masks is not None:
            for mask_data in results.masks.xy:
                mask_pts = mask_data.astype(np.int32)

                M = cv2.moments(mask_pts)
                if M['m00'] == 0:
                    continue

                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])

                if u < 0 or v < 0:
    	            continue

                if v >= depth_img.shape[0] or u >= depth_img.shape[1]:
                    continue

                depth = depth_img[v, u]
                if depth == 0:
                    continue

                cam_point = self.deproject(u, v, depth)
                base_point = self.transform_to_base(cam_point)

                if base_point is None:
                    continue

                pose = Pose()
                pose.position.x = float(base_point[0])
                pose.position.y = float(base_point[1])
                pose.position.z = float(base_point[2])
                pose.orientation.z = 0.0

                msg_out.poses.append(pose)

                # 시각화
                cv2.polylines(debug_img, [mask_pts], True, (0, 255, 0), 1)

        # publish only data
        if msg_out.poses:
            self.pose_pub.publish(msg_out)

        cv2.imshow("Chicken Detection", debug_img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ChickenDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
