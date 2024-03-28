import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
import time
from threading import Thread
from std_msgs.msg import String, UInt8
from can import Message, Bus

class JoyPub(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.Lpublisher_ = self.create_publisher(UInt8, 'ex_dt_left', 10)
        self.Rpublisher_ = self.create_publisher(UInt8, 'ex_dt_right', 10)

        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)
    def listener_callback(self, msg: Joy):
        uint8 = UInt8()

        if msg.axes[1] > 0: # up
            uint8.data = 150
            self.Lpublisher_.publish(uint8)
            self.get_logger().info("L: Up")
        elif msg.axes[1] < 0: # down 
            uint8.data = 50
            self.Lpublisher_.publish(uint8)
            self.get_logger().info("L: Down")
        if msg.axes[3] > 0:
            uint8.data = 150
            self.Rpublisher_.publish(uint8)
            self.get_logger().info("R: Up")
        elif msg.axes[3] < 0:
            uint8.data = 50
            self.Rpublisher_.publish(uint8)
            self.get_logger().info("R: Down")

    def timer_callback(self):
       pass 

def main(args=None):
    rclpy.init(args=args)

    joy_node = JoyPub()

    rclpy.spin(joy_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
