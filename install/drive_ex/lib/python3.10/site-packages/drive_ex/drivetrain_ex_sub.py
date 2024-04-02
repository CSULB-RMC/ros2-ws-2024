import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
#ignore can import error if it's there, it works if you installed python-can
import can


class DrivetrainExcavator(Node):
    
    def __init__(self):
        super().__init__('drivetrain_excavator')

        #publishes can messages of its current state periodically
        #currently every 0.02 seconds, so 50 times a second
        self.canSend = self.create_timer(0.02, self.can_callback)

        #create subscribers to listen for teleop computer commands
        self.ex_dt_left_sub = self.create_subscription(UInt8, 'ex_dt_left', self.ex_dt_left_update, 10)
        self.ex_dt_right_sub = self.create_subscription(UInt8, 'ex_dt_right', self.ex_dt_right_update, 10)
        self.ex_excavator_sub = self.create_subscription(UInt8, 'ex_excavator', self.ex_excavator_update, 10)
        self.ex_reg_sub = self.create_subscription(UInt8, 'ex_reg', self.ex_reg_update, 10)

        #create state variables, these keep track of what motors
        #should be running and how fast at the current moment
        self.ex_dt_left_speed = 0
        self.ex_dt_right_speed = 0
        self.ex_excavator_speed = 0
        self.ex_reg_speed = 0

        #create can bus link, right now is linked to virtual vcan 0, most likely
        #will be can0 when on the bot
        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='50000')

        self.i = 0
    
    def signal_conversion(self, msg_data: UInt8, bytes_range: int) -> list[int]:
        data: int = msg_data 
        # Forward msg correction
        if data > 100:
            data //= 2
        
        # covert controller signal to proper range (1000-100000)
        data *= 1000
        
        # convert signal to byte array
        temp_data: list[int] = []
        for i in range(bytes_range - 1, -1, -1):
            temp_data.append((data >> (8*i)) & 0xff)
        return temp_data 


    #updates the states of the left drivetrain motors
    def ex_dt_left_update(self, msg):
        #msg is an UInt8 from 0-200
        temp_data = self.signal_conversion(msg.data, 3)  
        # can message for right and left motor
        can_msg_m1 = can.Message(
                arbitration_id=0x15,
                data = temp_data, # place holder speed: 50%
                )
        can_msg_m2 = can.Message(
                arbitration_id=0x16,
                data = temp_data, # place holder speed: 50%
                )
        

        # Send to both left motors
        # self.get_logger().info(f'{self.ex_dt_left_speed}')
        # self.bus.send_periodic(can_msg_m1, self.ex_dt_left_speed)
        self.bus.send(can_msg_m1)
        self.bus.send(can_msg_m2)

        # Log Speed
        self.get_logger().info(f'{can_msg_m1}')
        # self.get_logger().info(f'{can_msg.data}, {controller_data}')

    #updates the states of the right drivetrains motors
    def ex_dt_right_update(self, msg):
        #msg is an UInt8 from 0-200
        # converts controller signal to bytes array
        temp_data = self.signal_conversion(msg.data, 3)  

        # self.get_logger().info('updating right ex dt')
       
        can_msg_m1 = can.Message(
                arbitration_id=0x17,
                data = temp_data,  # place holder speed: 50%
                )
        can_msg_m2 = can.Message(
                arbitration_id=0x18, # place holder speed: 50%
                data = temp_data, 
                )

        self.bus.send(can_msg_m1)
        self.bus.send(can_msg_m2)
        self.get_logger().info(f'{can_msg_m1}')

    def ex_excavator_update(self, msg):
        #msg is an UInt8 from 0-200
        # TODO
        # needs to start or stop procedure on button press
        temp_data = self.signal_conversion(msg.data, 3)

        self.ex_excavator_speed = msg.data        
        # self.get_logger().info('updating excavator')
        can_msg = can.Message(
                arbitration_id=0xe0,
                data = temp_data, 
                )

        self.bus.send(can_msg)
        self.get_logger().info(f'{can_msg}')


    def ex_reg_update(self, msg):
        #msg is an UInt8 from 0-200
        # TODO
        # needs start or stop procedure on button press
        self.ex_reg_speed = msg.data
        
        temp_data = self.signal_conversion(msg.data, 3)

        # self.get_logger().info('updating regolith')

        can_msg = can.Message(
                arbitration_id=0xb0,
                data = temp_data, 
                )


        self.bus.send(can_msg) 
        self.get_logger().info(f'{can_msg}')

    #periodically pushes out can messages to keep the rover running
    def can_callback(self):
        #TODO
        msg = can.Message(
            arbitration_id=0xC0FFEE,
            data=[0, 25, 0, 1, 3, 1, 4],
            is_extended_id=True
        )
        self.bus.send(msg)
        # self.get_logger().info(f'{msg.data}')
        self.i += 1
        pass

def main(args=None):
    print("Bus Publisher Active")
    rclpy.init(args=args)
    node = DrivetrainExcavator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
