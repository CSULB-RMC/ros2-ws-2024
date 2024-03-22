import socket
from canbus_interface.msg import CanFrame
from canbus_interface.srv import CanFrameSr
import rclpy                            
from rclpy.node import Node

class can_pub(can.Listener):
    def __init__(self,bus):
        self.bus = bus
        # whenever a message appears on the bus, this node will try to publish it in the ros network
        notifier = can.Notifier(bus, [self])


        self.canpub = self.create_subscription(Image)   
        self.cansrv = rospy.Service('send_frame', CanFrameSrv, self.send_message)

    def on_message_received(self,msg):
        #rospy.loginfo("received this %s"%(msg))
        canframe = CanFrame()
        canframe.timestamp = rospy.Time.now()
        canframe.arbitration_id = msg.arbitration_id
        canframe.data = [x for x in msg.data]
        #rospy.loginfo("sending this %s"%(canframe))
        self.canpub.publish(canframe)

    def send_message(self, req):
        #rospy.loginfo("sending message"%(req))
        #TODO implement this
        pass
if __name__ == '__main__':
    rclpy.init(args=args) 
    can_ifname =  rospy.get_param('can_ifname','can0')
    bus = can.interface.Bus(can_ifname)
    canros = can_pub(bus)
    r = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            r.sleep()
    except KeyboardInterrupt:
        bus.shutdown()    


