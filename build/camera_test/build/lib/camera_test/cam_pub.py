import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('topic_webcam_pub')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()
        self.i = 0
    
    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))
            self.i += 1
            self.get_logger().info(f'Publishing video frame {self.i}')

    def listener_callback(self, msg):
        if msg.buttons[3]:
            self.timer_callback()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        