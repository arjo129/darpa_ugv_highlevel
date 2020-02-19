#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice
from digi.xbee.io import IOLine

def estop():
    pub = rospy.Publisher('e_stop', String, queue_size=1)
    rospy.init_node('estop_receiver', anonymous=True)

    device = XBeeDevice("/dev/xbee", 9600)
    device.open()
    message = None
    print("E-Stop node has started")

    while True:
        # message = device.read_data()
        # if message:

        io_sample = device.read_io_sample()
        # io_line = IOLine.DIO1_AD1
        io_lines = []

        io_lines.append(IOLine.DIO0_AD0)
        io_lines.append(IOLine.DIO1_AD1)
        io_lines.append(IOLine.DIO2_AD2)
        io_lines.append(IOLine.DIO3_AD3)
        io_lines.append(IOLine.DIO4_AD4)

        for io_line in io_lines:
            if io_sample.has_digital_value(io_line):
                print("Line: " + str(io_line) + " is HIGH")

        # Check if the IO sample contains the expected IO line and value.
        # if io_sample.has_digital_value(io_line):
        #     stringType = str(message.data.decode("utf8"))
        #     print(stringType, rospy.get_time())
        #     msg = String()
        #     msg.data = stringType
        #     pub.publish(msg)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        estop()
    except rospy.ROSInterruptException:
        print("E-Stop node has stopped")
