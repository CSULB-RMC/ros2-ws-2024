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

        self.broker = 'ws://54.151.96.241:8083/mqtt'
        self.port = 8083
        self.topic = "python/mqtt"
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        self.username = 'porter.clevidence01@student.csulb.edu'
        self.password = '66b9ed8a-f2bc-4292-924c-466a06d9'


    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
        # For paho-mqtt 2.0.0, you need to add the properties parameter.
        # def on_connect(client, userdata, flags, rc, properties):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)
        # Set Connecting Client ID
        client = mqtt_client.Client(self.client_id)

        # For paho-mqtt 2.0.0, you need to set callback_api_version.
        # client = mqtt_client.Client(client_id=client_id, callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)

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

