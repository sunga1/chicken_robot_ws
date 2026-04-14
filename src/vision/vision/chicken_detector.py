import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseArray, Pose
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

class ChickenDetector(Node):
    def __init__(self):
        super().__init__('chicken_detector')
        
        self.bridge = CvBridge()
        
        model_path = os.path.join(os.path.expanduser('~'), 'Downloads', 'best.pt')
        self.model = YOLO(model_path)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pose_pub = self.create_publisher(PoseArray, '/detected_chickens', 10)
        
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_cb, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
            
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        
        
        self.latest_color = None
        self.latest_depth = None
        self.intrinsics = None
        
        self.create_timer(0.1, self.detection_loop)
        
        self.get_logger().info("ChickenDetector started")
        
        
    def info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = msg
            
    def color_cb(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        
    def deproject(self, u, v, depth):
        z = float(depth) * 0.001
        x = (u - self.intrinsics.k[2]) * z / self.intrinsics.k[0]
        y = (v - self.intrinsics.k[5]) * z / self.intrinsics.k[4]
        return np.array([x,y,z])
        
    def get_mat(self, t):
        pos = np.array([
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
        
        return pos, rot
        
        
    def detection_loop(self):
        if self.latest_color is None or self.latest_depth is None or self.intrinsics is None:
            return
            
        try:
            frame = self.latest_color.copy()
            depth_img = self.latest_depth
            
            result = self.model(fram, imgsz=480, conf=0.4)[0]
            
            pose_array = PoseArray()
            pose_array.header.frame_id = "base_link"
            pose_array.header.stamp = self.get_clock().now().to_msg()
            
            debug_img = frame.copy()
            
            try:
                now = rclpy.time.Time()
                t_b2l6 = self.tf_buffer.lookup_transform('base_link', 'link_6', now)
                t_l62c = self.tf_buffer.lookup_transform('link_6', 'camera_link', now)
                
                p_b2l6, r_b2l6 = self.get_mat(t_b2l6)
                p_l62c, r_l62c = self.get_mat(t_l62c)
                
            except:
                return
                
            if result.masks is not None:
                for mask_data in results.masks.xy:
                    mask_pts = mask_data.astype(np.int32)
                    
                    M = cv2.moment(mask_pts)
                    if M['m00'] == 0:
                        continue
                    
                    u = int(M['m10'] / M['m00'])
                    v = int(M['m01'] / M['m00'])
                    
                    if u < 0 or v < 0 or v >= depth_img.shape[0] or u >= depth_img.shape[1]:
                        continue
                        
                    depth = depth_img[v, u]
                    if depth == 0:
                        continue
                        
                    cam_point = self.deproject(u, v, depth)
                    base_point = r_b2l6 @ (r_l62c @ cam_point + p_l62c) + p_b2l6
                    
                    _, eig = cv2.PCACompute(mask_pts.astype(np.float32), mean=None)
                    angle = np.degrees(np.arctan2(eig[0][1]. eig[0][0])) + 90.0
                    
                    pose = Pose()
                    pose.position.x = float(base_point[0])
                    pose.position.y = float(base_point[1])
                    pose.position.z = float(base_point[2])
                    pose.orientation.z = float(angle)
                    
                    pose_array.poses.append(pose)
                    
                    cv2.polylines(debug_img, [mask_pts], True, (0,255,0), 2)
                    cv2.circle(debug_img, (u,v), 4, (0,0,255), -1)
                    
            if pose_array.poses:
                slef.pose_pub.publish(pose_array)
                self.get_logger().info(f"치킨 {len(pose_array.poses)}개 publish")
            
            cv2.imshow("Chicken Detection", debug_img)
            cv2.waitKey(1)
            
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            

def main():
    rclpy.init()
    node = ChickenDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    
    

if __name__ == '__main__':
    main()
    
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
