#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

from dsr_msgs2.srv import MoveLine, MoveStop
from ultralytics import YOLO

from pymodbus.client import ModbusTcpClient 


class ChickenChaser(Node):

    def __init__(self):
        super().__init__('chicken_chaser_node')

        # ------------------------------
        # OnRobot Gripper
        # ------------------------------

        self.GRIPPER_IP = "192.168.1.1"
        self.GRIPPER_UNIT = 65

        self.modbus_client = ModbusTcpClient(self.GRIPPER_IP, port=502)

        if self.modbus_client.connect():
            self.get_logger().info("Gripper Modbus Connected")
        else:
            self.get_logger().error("Gripper connection failed")

        # ------------------------------
        # Camera → Gripper transform
        # ------------------------------

        self.t_c2g = np.array([-0.04861006, 0.05671699, 0.16817704])

        self.q_c2g = [0.06996341, 0.01519766, 0.97633869, -0.20405127]

        self.R_c2g = R.from_quat(self.q_c2g).as_matrix()

        self.gripper_offset = 230.0

        # ------------------------------
        # YOLO model
        # ------------------------------

        home_dir = os.path.expanduser('~')
        model_path = os.path.join(home_dir, 'Downloads', 'best.pt')

        self.model = YOLO(model_path)

        # ------------------------------
        # Angle buffer
        # ------------------------------

        self.angle_buffer = []
        self.fixed_angle = 0.0

        # ------------------------------
        # ROS TF
        # ------------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------
        # Doosan Robot services
        # ------------------------------

        self.move_cli = self.create_client(MoveLine, '/motion/move_line')
        self.stop_cli = self.create_client(MoveStop, '/motion/move_stop')

        # ------------------------------
        # RealSense
        # ------------------------------

        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.profile = self.pipeline.start(config)

        self.intr = self.profile.get_stream(rs.stream.color)\
            .as_video_stream_profile().get_intrinsics()

        # ------------------------------

        self.create_timer(0.1, self.main_loop)

        self.get_logger().info(
            "[c] close | [o] open | [m] move | [d] down | [u] up | [q] quit")

    # ------------------------------------------------
    # Gripper control
    # ------------------------------------------------

    def set_gripper_modbus(self, mode):

        if mode == "OPEN":
            width, force, speed = 70, 20, 50
        else:
            width, force, speed = 10, 40, 50

        values = [int(width*10), force, speed, 1]

        try:

            self.modbus_client.write_registers(
                address=0,
                values=values,
                device_id=self.GRIPPER_UNIT
            )

            self.get_logger().info(f"Gripper {mode}")

        except Exception as e:

            self.get_logger().error(f"Gripper error: {e}")

    # ------------------------------------------------
    # PCA orientation
    # ------------------------------------------------

    def get_stable_angle(self, mask_coords):

        pts = mask_coords.astype(np.float32)

        mean, eigenvectors = cv2.PCACompute(pts, mean=None)

        angle = np.degrees(
            np.arctan2(eigenvectors[1, 1], eigenvectors[1, 0]))

        if angle < 0:
            angle += 180
            
        self.angle_buffer.append(angle)

        
        if len(self.angle_buffer) > 10:
            self.angle_buffer.pop(0)

        return np.mean(self.angle_buffer)

    # ------------------------------------------------
    # Main loop
    # ------------------------------------------------

    def main_loop(self):

        try:

            frames = self.pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return

            img = np.asanyarray(color_frame.get_data())

            now = rclpy.time.Time()

            trans = self.tf_buffer.lookup_transform(
                'base_link', 'link_6',
                now,
                rclpy.duration.Duration(seconds=0.1)
            )

            cur_t = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])

            robot_z_mm = cur_t[2] * 1000.0

            # ----------------------------
            # YOLO detection
            # ----------------------------

            results = self.model.predict(img, conf=0.5, verbose=False)

            b_point = None
            current_view_angle = 0

            for r in results:

                if r.masks is None:
                    continue

                mask_coords = r.masks.xy[0].astype(np.int32)

                cv2.polylines(img, [mask_coords], True, (0,255,0), 2)

                current_view_angle = self.get_stable_angle(mask_coords)

                M = cv2.moments(mask_coords)

                if M['m00'] == 0:
                    continue

                u_pix = int(M['m10']/M['m00'])
                v_pix = int(M['m01']/M['m00'])

                dist = depth_frame.get_distance(u_pix, v_pix)

                if dist == 0:
                    continue

                c_point = rs.rs2_deproject_pixel_to_point(
                    self.intr,
                    [u_pix, v_pix],
                    dist
                )

                cur_R = R.from_quat([
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w
                ]).as_matrix()

                g_point = self.R_c2g @ np.array(c_point) + self.t_c2g

                b_point = cur_R @ g_point + cur_t

                target_pick_z = (b_point[2]*1000.0) + self.gripper_offset

                cv2.circle(img,(u_pix,v_pix),5,(0,0,255),-1)

            # ----------------------------
            # Keyboard control
            # ----------------------------

            key = cv2.waitKey(1) & 0xFF

            if key == ord('c'):
                self.set_gripper_modbus("CLOSE")

            elif key == ord('o'):
                self.set_gripper_modbus("OPEN")

            elif key == ord('m') and b_point is not None:

                # robot yaw compensation
                robot_angle = 180 - current_view_angle + 90

                self.fixed_angle = robot_angle

                # move above chicken
                self.send_move_command(
                    [
                        b_point[0]*1000,
                        (b_point[1]+0.03)*1000,
                        robot_z_mm
                    ],
                    self.fixed_angle
                )

            elif key == ord('d') and b_point is not None:

                # approach downward
                self.send_move_command(
                    [
                        cur_t[0]*1000,
                        cur_t[1]*1000,
                        target_pick_z
                    ],
                    self.fixed_angle
                )

            elif key == ord('u'):

                self.send_move_command(
                    [
                        cur_t[0]*1000,
                        cur_t[1]*1000,
                        robot_z_mm+100
                    ],
                    self.fixed_angle
                )

            elif key == ord('q'):

                rclpy.shutdown()

            cv2.imshow("Chicken Control", img)

        except:
            pass

    # ------------------------------------------------

    def send_move_command(self, pos_mm, rz_deg):

        if not self.move_cli.wait_for_service(timeout_sec=1.0):
            return

        req = MoveLine.Request()

        req.pos = [
            pos_mm[0],
            pos_mm[1],
            pos_mm[2],
            0.0,
            180.0,
            float(rz_deg)
        ]

        req.vel = [60.0,60.0]
        req.acc = [60.0,60.0]

        self.move_cli.call_async(req)

    # ------------------------------------------------

    def send_stop_command(self):

        if not self.stop_cli.wait_for_service(timeout_sec=1.0):
            return

        req = MoveStop.Request()

        req.stop_mode = 1

        self.stop_cli.call_async(req)


# ------------------------------------------------

def main():

    rclpy.init()

    node = ChickenChaser()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.modbus_client.close()

        cv2.destroyAllWindows()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
