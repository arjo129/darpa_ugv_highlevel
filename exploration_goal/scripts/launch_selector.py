#!/usr/bin/env python

import rospy
import roslaunch
from sensor_msgs.msg import BatteryState
from rospy.exceptions import ROSException

import os

GUARANTEED_X1_TOPIC = '/X1/battery_state'
TOPIC_TIMEOUT = 10 # seconds
CURR_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
TEAMBASE_LAUNCH_URI = CURR_PATH + "/launch/teambase.launch"
X1_LAUNCH_URI = CURR_PATH + "/launch/x1.launch"
launch = None

def main():
    rospy.init_node('LaunchSelectorNode', anonymous=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    try:
        rospy.logwarn("[Launch-Selector] Waiting for message on {} for {} seconds \
                before deciding to launch X1 or teambase".format(GUARANTEED_X1_TOPIC, TOPIC_TIMEOUT))
        rospy.wait_for_message(GUARANTEED_X1_TOPIC, BatteryState, timeout=rospy.Duration(TOPIC_TIMEOUT))

        # X1 topic found, running X1 launch file
        launch = roslaunch.parent.ROSLaunchParent(uuid, [X1_LAUNCH_URI])
        rospy.loginfo("[Launch-Selector] Launching X1....")

    except ROSException:
        # Could not find X1 topic, so it is likely teambase docker. using Teambase launch file
        launch = roslaunch.parent.ROSLaunchParent(uuid, [TEAMBASE_LAUNCH_URI])
        rospy.loginfo("[Launch-Selector] Launching Teambase....")

    rospy.sleep(2.0)
    launch.start()
    rospy.loginfo("[Launch-Selector] Launch Succesful. All tasks completed.")

    rospy.spin()


if __name__ == "__main__":
    main()