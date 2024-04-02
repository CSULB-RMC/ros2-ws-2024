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
        self.ex_publisher_ = self.create_publisher(UInt8, 'ex_excavator', 10)
        self.reg_publisher_ = self.create_publisher(UInt8, 'ex_reg', 10)
        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)

    def listener_callback(self, msg: Joy):
        uint8 = UInt8()
        
        # Left Stick Maps - Left Drive Train
        if msg.axes[1] > 0.01: # L Stick Up 
            uint8.data = int((msg.axes[1] * 100) + 100) # add 100 to indicate forward motion and not include 100
            self.dt_l_publisher_.publish(uint8)
            self.get_logger().info(f'L-stick: Up, {uint8.data}')

        elif msg.axes[1] < 0: # L Stick Down 
            uint8.data = int((abs(msg.axes[1]) * 100) - 1) # subtract 1 to no include 100 
            self.dt_l_publisher_.publish(uint8)
            self.get_logger().info(f'L-stick: Down, {uint8.data}')
        
        else: # Left Drive Train Not Moving
            uint8.data = 100 # stop value 
            self.dt_l_publisher_.publish(uint8)

        # Right Stick Maps - Right Drive Train
        if msg.axes[4] > 0.01: # R Stick Up
            uint8.data = int((msg.axes[4] * 100) + 100) # add 100 to indicate forward motion and not include 100
            self.dt_r_publisher_.publish(uint8)
            self.get_logger().info(f'R-stick: Up, {uint8.data}')

        elif msg.axes[4] < 0: # R Stick Down
            uint8.data = int((abs(msg.axes[4]) * 100) - 1) # subtract 1 to no include 100 
            self.dt_r_publisher_.publish(uint8)
            self.get_logger().info(f'R-stick: Down, {uint8.data}')
        
        else: # Right Drive Train Not Moving
            uint8.data = 100 # stop value 
            self.dt_r_publisher_.publish(uint8)

        # D pad Maps - Excavator
        if msg.axes[7] > 0.01: # D pad Up
            uint8.data = 150
            self.ex_publisher_.publish(uint8)
            self.get_logger().info("Dpad: up")
            
        elif msg.axes[7] < 0: # D pad down
            uint8.data = 50
            self.ex_publisher_.publish(uint8)
            self.get_logger().info("Dpad: down")
        
        else:
            uint8.data = 100
            self.ex_publisher_.publish(uint8)

        # A button
        # TODO place holder 
        if msg.buttons[0] == 1:
            uint8.data = 150
            self.reg_publisher_.publish(uint8)
            self.get_logger().info("A: pressed")

        elif msg.buttons[0] == 0:
            uint8.data = 50
            self.reg_publisher_.publish(uint8)

    def timer_callback(self):
       pass 

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
