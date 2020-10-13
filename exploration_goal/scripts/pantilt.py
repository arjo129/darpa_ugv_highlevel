#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

if __name__ == "__main__":
    rospy.init_node("pan_tilt_driver")
    dur = rospy.Duration(1)
    camera_tilt = rospy.Publisher("pan_tilt/tilt_rate_cmd_double", Float64)
    rotate_up = Float64()
    rotate_up.data = 0.5
    rotate_down = Float64()
    rotate_down.data = -0.5
    camera_tilt.publish(rotate_up)
    rospy.sleep(dur)
    while not rospy.is_shutdown():
        camera_tilt.publish(rotate_down)
        rospy.sleep(dur)
        camera_tilt.publish(rotate_up)
        rospy.sleep(dur)
        