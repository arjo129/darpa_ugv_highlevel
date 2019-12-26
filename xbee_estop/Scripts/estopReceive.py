#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice

def estop():
    pub = rospy.Publisher('e_stop', String, queue_size=1)
    rospy.init_node('estop_receiver', anonymous=True)

    device = XBeeDevice("/dev/ttyUSB0", 9600)
    device.open()
    message = None
    print("E-Stop node has started")

    while True:
        message = device.read_data()
        if message:
            stringType = str(message.data.decode("utf8"))
            print(stringType, rospy.get_time())
            msg = String()
            msg.data = stringType
            pub.publish(msg)
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        estop()
    except rospy.ROSInterruptException:
        print("E-Stop node has stopped")
