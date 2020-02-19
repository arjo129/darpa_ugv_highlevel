#!/usr/bin/python3

import serial
import rospy
from std_msgs.msg import String
with serial.Serial('/dev/xbee', 9600, timeout=1) as ser:
    def reset(msg):
        ser.write(b'\x7E\x00\x05\x08\x01\x44\x31\x04\x7D')
        ser.read(40)
    rospy.init_node("estop_xbee")
    pub = rospy.Publisher("/e_stop", String)
    sub = rospy.Subscriber("/start", String, reset)
    while not rospy.is_shutdown():
        ser.write(b'\x7E\x00\x04\x08\x01\x49\x53\x5A')
        if ser.read(28) == b'~\x00\x0b\x88\x01IS\x00\x01\x00\x02\x00\x00\x02\xd5':
            st = String()
            st.data = "hi"
            pub.publish(st)
            print("estopped")

