import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
#ignore can import error if it's there, it works if you installed python-can
import random
from paho.mqtt import client as mqtt_client
import can
class DB_Broker(Node):
    
    def __init__(self):
        super().__init__('db_broker_pub')
        # python 3.11
        self.broker = 'broker.emqx.io'
        self.port = 1883
        self.topic = "python/mqtt"
        # Generate a Client ID with the subscribe prefix.
        self.client_id = 'mini_bot_broker'
        #self.username = 'porter.clevidence01@student.csulb.edu'
        #self.password = '1115b07a-9f83-4422-918f-f2caaf83'


    def connect_mqtt(self) -> mqtt_client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(self.client_id)
        client.username_pw_set(self.username, self.password)
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        return client


    def publish(self, client): 
        while True:
            result = client.publish(self.topic, "hello")
            status = result[0]
            if status == 0:
                print(f"Send Hello to topic `{self.topic}`")
            else:
                print(f"Failed to send message to topic {self.topic}")

def main(args=None):
    print("DB")

    rclpy.init(args=args)
    db_node = DB_Broker()
    client = db_node.connect_mqtt()
    client.loop_start()
    db_node.publish(client)
    client.loop_forever()

    rclpy.spin(db_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    db_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

