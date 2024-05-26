import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
#ignore can import error if it's there, it works if you installed python-can
import random
from paho.mqtt import client as mqtt_client
import can
import time
class DB_Broker(Node):
    
    def __init__(self):
        super().__init__('db_broker_pub')
         
        self.status_timer = self.create_timer(1, self.timer_callback)
        self.broker = '54.151.96.241'
        # self.broker = 'broker.emqx.io'
        self.port = 8083
        # self.port = 1883
        self.topic = 'python/mqtt'
        # self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        self.username = 'porter.clevidence01@student.csulb.edu'
        self.password = '66b9ed8a-f2bc-4292-924c-466a06d9'

        self.FIRST_RECONNECT_DELAY = 1
        self.RECONNECT_RATE = 2
        self.MAX_RECONNECT_COUNT = 12
        self.MAX_RECONNECT_DELAY = 60
 
    # def on_connect(client, userdata, flags, rc):
    # For paho-mqtt 2.0.0, you need to add the properties parameter.
    def on_connect(self, client, userdata, flags, rc, properties):
        self.get_logger().info('connecting...')
        if rc == 0:
            self.get_logger().info(f'Connected to MQTT Broker!')
        else:
            self.get_logger().info(f'Failed to connect, return code {rc}\n')

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().info("Disconnected with result code: %s" % rc)
        reconnect_count, reconnect_delay = 0, self.FIRST_RECONNECT_DELAY
        while reconnect_count < self.MAX_RECONNECT_COUNT:
            self.get_logger().info("Reconnecting in %d seconds..." % reconnect_delay)
            time.sleep(reconnect_delay)

            try:
                client.reconnect()
                self.get_logger().info("Reconnected successfully!")
                return
            except Exception as err:
                self.get_logger().info("%s. Reconnect failed. Retrying..." % err)

            reconnect_delay *= self.RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, self.MAX_RECONNECT_DELAY)
            reconnect_count += 1
        self.get_logger().info("Reconnect failed after %s attempts. Exiting..." % reconnect_count)
    
    def publish(self, client):
        msg_count = 1
        while True:
            time.sleep(1)
            msg = f"messages: {msg_count}"
            result = client.publish(self.topic, msg)
            # result: [0, 1]
            status = result[0]
            if status == 0:
                print(f"Send `{msg}` to topic `{self.topic}`")
            else:
                print(f"Failed to send message to topic {self.topic}")
            msg_count += 1
            if msg_count > 5:
                break
    
    def connect_mqtt(self):
        # Set Connecting Client ID
        # client = mqtt_client.Client(client_id='')

        # For paho-mqtt 2.0.0, you need to set callback_api_version.
        client = mqtt_client.Client(transport="websockets", callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)
        # client = mqtt_client.Client(callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)

        client.username_pw_set(self.username, self.password)
        client.on_connect = self.on_connect
        client.connect(self.broker, self.port)
        return client

    def timer_callback(self):
        client = self.connect_mqtt()
        client.loop_start()
        self.publish(client)
        client.loop_stop() 

def main(args=None):
    print("DB1")

    rclpy.init(args=args)
    db_node = DB_Broker()


    rclpy.spin(db_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    db_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

