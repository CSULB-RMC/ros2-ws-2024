import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String
#ignore can import error if it's there, it works if you installed python-can

class UI(Node):
    
    def __init__(self):
        super().__init__('ui_health')

        self.status_timer = self.create_timer(1, self.timer_callback)
        self.status_timer_fail = self.create_timer(5, self.timer_fail_callback)
        self.cb_sub = self.create_subscription(String, 'cb_active', self.cb_ui_update, 10)
        self.cb_sub = self.create_subscription(String, 'ex_active', self.ex_ui_update, 10)
        self.cb_health = "Dead"
        self.ex_health = "Dead"

    def cb_ui_update(self, msg):
        self.cb_health = msg.data

    def ex_ui_update(self, msg):
        self.ex_health = msg.data

    def timer_callback(self):
        print(chr(27) + "[2J")
        print(f'Excavator Bot: {self.ex_health}')
        print(f'Cargo Bot: {self.cb_health}')
    
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
