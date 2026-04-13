import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading

class KeyInput(Node):
    def __init__(self):
        super().__init__('key_input')
        self.pub = self.create_publisher(Bool, '/robot/confirm', 10)

        # 🔥 input은 별도 thread로
        thread = threading.Thread(target=self.input_loop, daemon=True)
        thread.start()

    def input_loop(self):
        while rclpy.ok():
            input("Enter 누르면 이동\n")
            msg = Bool()
            msg.data = True
            self.pub.publish(msg)
            self.get_logger().info("confirm sent")

def main():
    rclpy.init()
    node = KeyInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
