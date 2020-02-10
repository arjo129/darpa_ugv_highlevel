#!/usr/bin/env python3
import rospy
import time
from digi.xbee.devices import XBeeDevice

device = XBeeDevice("/dev/ttyUSB0", 9600)
device.open()
print("SENDING ESTOP SIGNAL")
for i in range(1,10):
    device.send_data_broadcast("ESTOP")
    print(i,"Sent")
device.close()