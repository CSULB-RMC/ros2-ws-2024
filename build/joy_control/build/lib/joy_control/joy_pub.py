import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
import time
from threading import Thread
from std_msgs.msg import String, UInt8
from can import Message, Bus
# test
class JoyPub(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.dt_l_publisher_ = self.create_publisher(UInt8, 'ex_dt_left', 10)
        self.dt_r_publisher_ = self.create_publisher(UInt8, 'ex_dt_right', 10)
        self.ex_conveyer_publisher_ = self.create_publisher(UInt8, 'ex_conveyer', 10)
        self.ex_arm_publisher_ = self.create_publisher(UInt8, 'ex_arm', 10)
        self.ex_digger_publisher_ = self.create_publisher(UInt8, 'ex_digger', 10)
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
            uint8.data = int((msg.axes[1] * 100) + 100) # add 100 to indicate forward motion and not include 100
            self.dt_l_publisher_.publish(uint8)

        elif msg.axes[1] < -self.DEADBAND: # L Stick Down 
            uint8.data = int((abs(msg.axes[1]) * 100)) # subtract 1 to no include 100 
            self.dt_l_publisher_.publish(uint8)

        else:
            uint8.data = 0 # deadband resets it to neutral
            self.dt_l_publisher_.publish(uint8)

        # Right Stick Maps - Right Drive Train
        if msg.axes[3] > self.DEADBAND: # R Stick Up
            uint8.data = int((msg.axes[3] * 100) + 100) # add 100 to indicate forward motion and not include 100
            self.dt_r_publisher_.publish(uint8)

        elif msg.axes[3] < -self.DEADBAND: # R Stick Down
            uint8.data = int((abs(msg.axes[3]) * 100)) # subtract 1 to no include 100 
            self.dt_r_publisher_.publish(uint8)
            
        else:
            uint8.data = 0 # deadband resets it to neutral
            self.dt_r_publisher_.publish(uint8)
        
        # D pad Maps - Excavator
        if msg.axes[5] > 0.01: # D pad Up
            uint8.data = 110
            self.ex_conveyer_publisher_.publish(uint8)
            
        elif msg.axes[5] < 0: # D pad down
            uint8.data = 10
            self.ex_conveyer_publisher_.publish(uint8)
        else:
            uint8.data = 0
            self.ex_conveyer_publisher_.publish(uint8)

        # A button
        if msg.buttons[2] == 1:
            uint8.data = 10
            self.ex_digger_publisher_.publish(uint8)
        else:
            uint8.data = 0
            self.ex_digger_publisher_.publish(uint8)
        
        

def main(args=None):
    print("Controller Active")
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
