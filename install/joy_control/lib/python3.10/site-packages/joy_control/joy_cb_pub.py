import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
import time
from threading import Thread
from std_msgs.msg import String, UInt8
from can import Message, Bus
# test
class JoyPub_Cb(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.cb_l_publisher_ = self.create_publisher(UInt8, 'cb_dt_left', 10)
        self.cb_r_publisher_ = self.create_publisher(UInt8, 'cb_dt_right', 10)
        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)
        
        self.DEADBAND = 0.05

    def listener_callback(self, msg: Joy):
        uint8 = UInt8()
        
        # Left Stick Maps - Left Drive Train
        if msg.axes[1] > self.DEADBAND: # L Stick Up 
            uint8.data = 50 - int((msg.axes[1] * 100) // 2) # add 100 to indicate forward motion and not include 100
            self.cb_l_publisher_.publish(uint8)
        elif msg.axes[1] < -self.DEADBAND: # L Stick Down 
            uint8.data = 50 + int((abs(msg.axes[1]) * 100) // 2) # subtract 1 to no include 100 
            self.cb_l_publisher_.publish(uint8)
        else:
            uint8.data = 1 # deadband resets it to neutral
            self.cb_l_publisher_.publish(uint8)

        # Right Stick Maps - Right Drive Train
        if msg.axes[3] > self.DEADBAND: # R Stick Up
            uint8.data = 50 - int((msg.axes[3] * 100) // 2) # add 100 to indicate forward motion and not include 100
            self.cb_r_publisher_.publish(uint8)
        elif msg.axes[3] < -self.DEADBAND: # R Stick Down
            uint8.data = 50 + int((abs(msg.axes[3]) * 100) // 2) # subtract 1 to no include 100 
            self.cb_r_publisher_.publish(uint8)
        else:
            uint8.data = 1 # deadband resets it to neutral
            self.cb_r_publisher_.publish(uint8)    

def main(args=None):
    print("Controller Active")
    rclpy.init(args=args)

    joy_node = JoyPub_Cb()

    rclpy.spin(joy_node)
    x = String()
    x.data = 'Dead'
    joy_node.activity_publisher_.publish(x)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
