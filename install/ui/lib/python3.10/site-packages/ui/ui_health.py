import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String
import os
#ignore can import error if it's there, it works if you installed python-can

class UI(Node):
    
    def __init__(self):
        super().__init__('ui_health')

        self.status_timer = self.create_timer(1, self.timer_callback)
        self.status_timer_fail = self.create_timer(5, self.timer_fail_callback)
        self.cb_sub = self.create_subscription(String, 'cb_active', self.cb_ui_update, 10)
        self.cb_sub = self.create_subscription(String, 'ex_active', self.ex_ui_update, 10)
        
        self.ex_dt_left_sub = self.create_subscription(UInt8, 'ex_dt_left', self.ex_dt_left_update, 10)
        self.ex_dt_right_sub = self.create_subscription(UInt8, 'ex_dt_right', self.ex_dt_right_update, 10)
        self.ex_conveyer_sub = self.create_subscription(UInt8, 'ex_conveyer', self.ex_conveyer_update, 10)
        self.ex_arm_sub = self.create_subscription(UInt8, 'ex_arm', self.ex_arm_update, 10)
        self.ex_digger_sub = self.create_subscription(UInt8, 'ex_digger', self.ex_digger_update, 10)
        self.ex_servo_sub = self.create_subscription(UInt8, 'ex_servo', self.ex_servo_update, 10)
        self.cb_dt_left_sub = self.create_subscription(UInt8, 'cb_dt_left', self.cb_dt_left_update, 10)
        self.cb_dt_right_sub = self.create_subscription(UInt8, 'cb_dt_right', self.cb_dt_right_update, 10)
        self.cb_scoop_sub = self.create_subscription(UInt8, 'cb_scoop', self.cb_scoop_update, 10)



        self.cb_health = "Dead"
        self.ex_health = "Dead"
        self.health_msg = "" 
        self.cb_dt_left_speed = 0
        self.cb_dt_right_speed = 0
        self.cb_scoop_speed = 0
        self.ex_dt_left_speed = 0
        self.ex_dt_right_speed = 0
        self.ex_conveyer_speed = 0
        self.ex_arm_speed = 0
        self.ex_digger_speed = 0
        self.ex_servo_speed = 0



    def ex_dt_left_update(self, msg):
        self.ex_dt_left_speed = msg.data

    def ex_dt_right_update(self, msg):
        self.ex_dt_right_speed = msg.data

    def ex_arm_update(self, msg):
        self.ex_arm_speed = msg.data

    def ex_digger_update(self, msg):
        self.ex_digger_speed = msg.data
    
    def ex_servo_update(self, msg):
        self.ex_servo_speed = msg.data

    def cb_dt_left_update(self, msg):
        self.cb_dt_left_speed = msg.data

    def cb_dt_right_update(self, msg):
        self.cb_dt_right_speed = msg.data  

    def cb_ui_update(self, msg):
        self.cb_health = msg.data
    
    def cb_scoop_update(self, msg):
        self.cb_scoop_speed = msg.data

    def ex_ui_update(self, msg):
        self.ex_health = msg.data

    def ex_conveyer_update(self, msg):
        self.ex_conveyer_speed = msg.data

    def generate_ascii_art(self, text):
        # Define ASCII art characters for each letter        
        ascii_art = {
            'L': ["#    ", "#    ", "#    ", "#    ", "#####"],
            'U': ["#   #", "#   #", "#   #", "#   #", "#####"],
            'N': ["#   #", "##  #", "# # #", "#  ##", "#   #"],
            'A': ["  #  ", " # # ", "#####", "#   #", "#   #"],
            'B': ["#### ", "#   #", "#### ", "#   #", "#### "],
            'O': ["#####", "#   #", "#   #", "#   #", "#####"],
            'T': ["#####", "  #  ", "  #  ", "  #  ", "  #  "],
            'I': ["#####", "  #  ", "  #  ", "  #  ", "#####"],
            'C': ["#####", "#    ", "#    ", "#    ", "#####"],
            'S': ["#####", "#    ", "#####", "    #", "#####"]
        }

        # Convert input text to uppercase
        text = text.upper()

        # Print ASCII art for each letter in the text
        for row in range(5):
            for char in text:
                print(ascii_art.get(char, ["     "])[row], end="  ")
            print()

    def timer_callback(self):
        # Call the function with the text "LUNABOTICS"
        os.system('cls' if os.name == 'nt' else 'clear')
        self.generate_ascii_art("LUNABOTICS")
        s = f'Excavator Bot: {self.ex_health}, '
        s += f'right_dt: {self.ex_dt_right_speed} left_dt: {self.ex_dt_left_speed}, '
        s += f'conveyer speed: {self.ex_conveyer_speed}, '
        s += f'arm speed: {self.ex_arm_speed}, '
        s += f'digger speed: {self.ex_digger_speed},'
        s += f'servo speed: {self.ex_servo_speed}'
        print(s)

        s2 = f'Cargo Bot: {self.cb_health}, '
        s2 += f'right_dt: {self.cb_dt_right_speed} left_dt: {self.cb_dt_left_speed}, '
        s2 += f'conveyer speed: {self.cb_scoop_speed}, '
        print(s2)
    
    def timer_fail_callback(self):
        self.cb_health = "Dead"
        self.ex_health = "Dead"


def main(args=None):
    rclpy.init(args=args)
    node = UI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
