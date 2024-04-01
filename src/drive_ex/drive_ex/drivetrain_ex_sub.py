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
    
    #updates the states of the left drivetrain motors
    def ex_dt_left_update(self, msg):
        #msg is an UInt8 from 0-200
        #TODO
        # self.get_logger().info('updating left ex dt')
        self.ex_dt_left_speed = msg.data.to_bytes(8, 'big')

        can_msg = can.Message(
                arbitration_id=0xd0,
                data = self.ex_dt_left_speed,
                )
        
        self.bus.send(can_msg)
        self.get_logger().info(f'{can_msg}')
        # self.get_logger().info(f'{msg.data}, {controller_data}')

    #updates the states of the right drivetrains motors
    def ex_dt_right_update(self, msg):
        #msg is an UInt8 from 0-200
        #TODO

        self.ex_dt_right_speed =  msg.data.to_bytes(8, 'big')
        # self.get_logger().info('updating right ex dt')
       
        can_msg = can.Message(
                arbitration_id=0xd1,
                data = self.ex_dt_right_speed 
                )

        self.bus.send(can_msg)
        self.get_logger().info(f'{can_msg}')

    def ex_excavator_update(self, msg):
        #msg is an UInt8 from 0-200
        #TODO
        self.ex_excavator_speed = msg.data.to_bytes(8, 'big')
        # self.get_logger().info('updating excavator')
        
        can_msg = can.Message(
                arbitration_id=0xe0,
                data = self.ex_excavator_speed 
                )

        self.bus.send(can_msg)
        self.get_logger().info(f'{can_msg}')


    def ex_reg_update(self, msg):
        #msg is an UInt8 from 0-200
        #TODO
        self.ex_reg_speed = msg.data.to_bytes(8, 'big')
        # self.get_logger().info('updating regolith')

        can_msg = can.Message(
                arbitration_id=0xb0,
                data = self.ex_reg_speed 
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

        
