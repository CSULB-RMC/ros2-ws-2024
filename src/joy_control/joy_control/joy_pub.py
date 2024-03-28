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
        self.dt_left_publisher_ = self.create_publisher(UInt8, 'ex_dt_left', 10)
        self.dt_r_publisher_ = self.create_publisher(UInt8, 'ex_dt_right', 10)
        self.ex_up_publisher_ = self.create_publisher(UInt8, 'ex_up', 10)
        self.ex_down_publisher_ = self.create_publisher(UInt8, 'ex_down', 10)

        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)

    def listener_callback(self, msg: Joy):
        uint8 = UInt8()
        
        # Left Stick Maps - Left Drive Train
        if msg.axes[1] > 0: # L Stick Up
            uint8.data = 150
            self.dt_left_publisher_.publish(uint8)
            self.get_logger().info("L-stick: Up")

        elif msg.axes[1] < 0: # L Stick Down 
            uint8.data = 50
            self.dt_left_publisher_.publish(uint8)
            self.get_logger().info("L-stick: Down")
        
        # Right Stick Maps - Right Drive Train
        if msg.axes[3] > 0: # R Stick Up
            uint8.data = 150
            self.dt_r_publisher_.publish(uint8)
            self.get_logger().info("R-stick: Up")

        elif msg.axes[3] < 0: # R Stick Down
            uint8.data = 50
            self.dt_r_publisher_.publish(uint8)
            self.get_logger().info("R-stick: Down")
        
        # D pad Maps - Excavator
        if msg.buttons[11] > 0: # D pad Up
            uint8.data = 150
            self.ex_up_publisher_.publish(uint8)
            self.get_logger().info("Dpad: up")
            
        elif msg.buttons[12] > 0: # D pad down
            uint8.data = 50
            self.ex_down_publisher_.publish(uint8)
            self.get_logger().info("Dpad: down")

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