import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
#ignore can import error if it's there, it works if you installed python-can
import can


class DrivetrainExcavator(Node):
    
    def __init__(self):
        super().__init__('drivetrain_excavator')

        # publishes can messages of its current state periodically
        # currently every 0.02 seconds, so 50 times a second
        # ...
        # changing timer doesn't effect duty cycle speed
        # self.canSend = self.create_timer(1, self.can_callback)

        # create subscribers to listen for teleop computer commands
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
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate='500000')

        self.i = 0
    
    # Converts Controller Speed to byte array (decimal form)
    # Alg: signal -> percentage * 1000 (UInt16) -> Hexadecimal Byte Form -> Decimal Byte Form 
    # Ex. 200 -> 50% -> 50,000 = [80, 200]
    def signal_conversion(self, msg_data: int, bytes_range: int) -> list[int]:
        data: int = msg_data 
        temp_data: list[int] = []

        #make sure the direction is correct
        if data > 100:
            #increment for 2's comp
            c = 1
            # Forward msg correction:
            data -= 100
            # covert controller signal to proper range (1000-100000)
            data *= 1000

            #convert to byte array but also 2's compliment to reverse motor
            for i in range(bytes_range - 1, -1, -1):
                temp_data.append(255 - ((data >> (8*i)) & 0xff))

            for i in range(len(temp_data) - 1, - 1, -1):
                temp_data[i] += c 
                if temp_data[i] > 255:
                    temp_data[i] = 0
                else:
                    c = 0
                    break


        else:
            # covert controller signal to proper range (1000-100000)
            data *= 1000
    
            # convert signal to byte array
            for i in range(bytes_range - 1, -1, -1):
                temp_data.append((data >> (8*i)) & 0xff)
        
        return temp_data 


    #updates the states of the left drivetrain motors
    def ex_dt_left_update(self, msg):
        # checks if speed is different then previous message published
        if self.ex_dt_left_speed == msg.data:
            return None
        self.ex_dt_left_speed = msg.data

        temp_data = self.signal_conversion(self.ex_dt_left_speed, 4)  
        # can message for right and left motor
        can_msg_m1 = can.Message(
                arbitration_id = 16,
                data = temp_data, # place holder 
                is_extended_id = True
                )
        # Changed so both motors are active on 16
        #can_msg_m2 = can.Message(
        #        arbitration_id = 15,
        #        data = [32], # place holder
        #        is_extended_id = True
        #        )
        

        # Send to Both Left Motors
        self.bus.send(can_msg_m1)
        # self.bus.send(can_msg_m2)

        # Log Can Message
        # self.get_logger().info(f'{can_msg_m1}')


    #updates the states of the right drivetrains motors
    def ex_dt_right_update(self, msg):
        # checks if speed is different then previous message published
        if self.ex_dt_right_speed == msg.data:
            return None
        self.ex_dt_right_speed = msg.data

        # converts controller signal to bytes array
        temp_data = self.signal_conversion(self.ex_dt_right_speed, 4)  

        can_msg_m1 = can.Message(
                arbitration_id = 17,
                data = temp_data,  # place holder speed: 50%
                is_extended_id = True,
                )
        can_msg_m2 = can.Message(
                arbitration_id =18, 
                data = temp_data, # place holder speed: 50%
                is_extended_id = True
                )
        
        # Send to Both Right Motors
        self.bus.send(can_msg_m1)
        self.bus.send(can_msg_m2)

        # Log Can Message
        # self.get_logger().info(f'{can_msg_m1}')
    
    # Sample Dpad Control Scheme
    def ex_excavator_update(self, msg):
        #msg is an UInt8 from 0-200
        # TODO
        # needs correct start or stop procedure on button press
        temp_data = self.signal_conversion(msg.data, 4)

        self.ex_excavator_speed = msg.data 
        # self.get_logger().info('updating excavator')
        can_msg = can.Message(
                arbitration_id=16,
                data = [], 
                )
        
        # Send Message to Excavator Motor
        self.bus.send(can_msg)
        # self.get_logger().info(f'{can_msg}')

    # Sample A Button Scheme
    def ex_reg_update(self, msg):
        #msg is an UInt8 from 0-200
        # TODO
        # needs corrent start or stop procedure on button press

        self.ex_reg_speed = self.signal_conversion(100, 4)
        
        temp_data = self.signal_conversion(msg.data, 4)

        can_msg = can.Message(
                arbitration_id=0xb0,
                data = temp_data, 
                )

        # Send Message to Reg Motor
        self.bus.send(can_msg) 
        # self.get_logger().info(f'{can_msg}')

    #Demo Function that calls motor 16 and 17, one command turns motor on.
    def can_callback(self):
        #TODO
        can_msg_1 = can.Message(
            arbitration_id=16, # motor id
            data=[],         # didn't test if this changed motor speed
            is_extended_id=True 
            )
        can_msg_2 = can.Message(
            arbitration_id=17,
            data=[32],
            is_extended_id=True
            )
        self.bus.send(can_msg_1)
        # self.bus.send(can_msg_2)
        

def main(args=None):
    print("Bus Publisher Active")
    rclpy.init(args=args)
    node = DrivetrainExcavator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
