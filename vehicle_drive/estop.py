#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def cmd_vel_forwarder(message):
    global estopped
    if not estopped:
        cmd_vel_pub.publish(message)

def on_estop_cancelled(message):
    global estopped
    estopped = False

def on_estop_triggered(message):
    global estopped
    estopped = True



rospy.init_node("estop_muxer")
cmd_vel_pub = rospy.Publisher("/controller/cmd_vel", Twist)
cmd_vel_lister = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_forwarder)
start_sub = rospy.Subscriber("/start", String, on_estop_cancelled)
stop_sub = rospy.Subscriber("/e_stop", String, on_estop_triggered)
global estopped
estopped = False

rospy.spin()
