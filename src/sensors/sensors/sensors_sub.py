import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String
#ignore can import error if it's there, it works if you installed python-can
import can

class SensorSub(Node):
    def __init__(self):
        super().__init__('sensors_sub')
        self.status_timer = self.create_timer(1, self.timer_callback)
        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='500000')

    def timer_callback(self):
        for msg in self.bus:
            print(msg)

def main(args=None):
    print("Sensor Sub Active")
    rclpy.init(args=args)
    node = SensorSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
