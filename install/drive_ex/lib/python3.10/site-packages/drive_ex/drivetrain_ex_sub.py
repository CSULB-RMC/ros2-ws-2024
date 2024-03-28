import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
import can


class DrivetrainExcavator(Node):
    
    def __init__(self):
        super().__init__('drivetrain_excavator')
        self.canSend = self.create_timer(0.02, self.can_callback)

        self.ex_dt_left_sub = self.create_subscription(UInt8, 'ex_dt_left', self.ex_dt_left_update, 10)
        self.ex_dt_right_sub = self.create_subscription(UInt8, 'ex_dt_right', self.ex_dt_right_update, 10)

        self.ex_dt_left_speed = 0
        self.ex_dt_right_speed = 0

        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='50000')

        self.i = 0

    def ex_dt_left_update(self, msg):
        #TODO
        self.get_logger().info('updating left ex dt')
        pass

    def ex_dt_right_update(self, msg):
        #TODO
        self.get_logger().info('updating right ex dt')
        pass

    def can_callback(self):
        #TODO
        msg = can.Message(
            arbitration_id=0xC0FFEE,
            data=[0, 25, 0, 1, 3, 1, 4, self.i],
            is_extended_id=True
        )
        self.bus.send(msg)
        self.i += 1
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DrivetrainExcavator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
