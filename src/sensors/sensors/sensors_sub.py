import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String, Int32, Int16
#ignore can import error if it's there, it works if you installed python-can
import can

class SensorSub(Node):
    def __init__(self):
        super().__init__('sensors_sub')

        filters=[
                {"can_id": 0x0000100F, "can_mask": 0x1FFFFFFF, "extended": True}
        ]
        # Standard CAN
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate='500000', can_filters=filters)
        
        self.vesc1_temp_publisher_ = self.create_publisher(Int16, 'temp_pub', 10) 
        self.status_timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        for msg in self.bus:
            num = int.from_bytes(msg.data[2:4], byteorder='big', signed=False)
            temp = Int16()
            
            min = -32768
            max = 32767

            self.get_logger().info(f'{num}')
            if num < min or num > max:
                num = ((num + 2**15) % 2**16) - 2**15
            
            self.get_logger().info(f'{num}')
            temp.data = num
            print("publishing ", temp)
            self.vesc1_temp_publisher_.publish(temp)

def main(args=None):
    print("Sensor Sub Active")
    rclpy.init(args=args)
    node = SensorSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
