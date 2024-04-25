import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge
import cv2, time
from threading import Thread


class ImagePublisher(Node):
    
    def __init__(self):
        super().__init__('topic_webcam_pub_2')
        self.publisher_ = self.create_publisher(Image, 'image_raw_2', 10)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('P', 'N', 'G', ' '))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cv_bridge = CvBridge()
        self.i = 0
        
        # FPS = 1/X
        # X = desired FPS
        self.FPS = 1/30
        self.FPS_MS = int(self.FPS * 1000)

        self.thread = Thread(target = self.update, args = ())
        self.thread.daemon = True
        self.thread.start()

        self.status = False
        self.frame  = None

    def update(self):
        while True:
            if self.cap.isOpened():
                (self.status, self.frame) = self.cap.read()
            time.sleep(self.FPS)

    def timer_callback(self):
        self.cap.read()
        self.status, self.frame = self.cap.read()

        if self.status == True:
            self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(self.frame, 'bgr8'))
            self.i += 1
            self.get_logger().info(f'Publishing video frame {self.i}')

    def listener_callback(self, msg):
        if msg.buttons[2]:
            self.timer_callback()

def main(args=None):
    print("Camera On")
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
