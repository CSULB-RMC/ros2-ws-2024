import rclpy
from rclpy.node import Node
from paho.mqtt import client as mqtt_client
import paho.mqtt.publish as publish
import time
from std_msgs.msg import UInt8, String, Int16
from dotenv import find_dotenv, load_dotenv
import os
load_dotenv(find_dotenv())
class DB_Broker(Node):
    
    def __init__(self):
        super().__init__('db_broker_pub')
         
        # self.status_timer = self.create_timer(5, self.timer_callback)
        self.vesc1_temp_sub = self.create_subscription(Int16, 'temp_pub', self.temp_update, 10)
        
        self.temp = 0

        self.broker = os.getenv("DATANIZ_IP")
        self.port = 1883
        self.topic = 'python/mqtt'
        self.username = os.getenv("DATANIZ_EMAIL")
        self.password = os.getenv("DATANIZ_KEY") 

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

    def on_publish(self, client, userdata, mid, reason_code, properties):
        print("mid: "+str(userdata))

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
    
    def connect_mqtt(self):
        # Set Connecting Client ID
        # client = mqtt_client.Client(client_id='')

        # For paho-mqtt 2.0.0, you need to set callback_api_version.
#        client = mqtt_client.Client(transport="websockets")
        # client = mqtt_client.Client()
        client = mqtt_client.Client(callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)

        client.username_pw_set(self.username, self.password)
        client.on_connect = self.on_connect
        client.on_publish = self.on_publish
        client.connect(self.broker, self.port)
        return client

    def temp_update(self, msg):
        client = self.connect_mqtt()
        client.loop_start()
        temp = msg.data
        self.get_logger().info(f"Sending {temp}")
        client.publish(self.topic, temp)
        client.loop_stop() 

def main(args=None):
    print("DB")

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

