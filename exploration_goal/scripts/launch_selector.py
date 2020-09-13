#!/usr/bin/env python

import rospy
import roslaunch
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Bool
from rospy.exceptions import ROSException

import os

GUARANTEED_X1_TOPIC = '/X1/battery_state'
TOPIC_TIMEOUT = 10 # seconds
SUBT_START_SRV_NAME = "/subt/start"

CURR_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
LAUNCH_FILE_PATHS = CURR_PATH+"/launch"
TEAMBASE_LAUNCH_URI = CURR_PATH + "/launch/teambase.launch"
TEAMBASE_NAME= "teambase"
launch = None

def main():
    rospy.init_node('LaunchSelectorNode', anonymous=True)
    rospy.logwarn("[Launch-Selector] Waiting for /subt/start service to start scoring...")
    rospy.wait_for_service(SUBT_START_SRV_NAME)
    
    # Start the scoring server
    subt_start_srv = rospy.ServiceProxy(SUBT_START_SRV_NAME, SetBool)
    start_req = SetBoolRequest()
    start_req.data = True
    subt_start_srv(start_req)
    rospy.loginfo("[Launch-Selector] Subt Scoring server started")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)


    #Get all publishe topics, map them to a corresponding launch file
    # /X1/some_topic gets mapped to x1.launch
    topics = rospy.get_published_topics()

    launch_files = os.listdir(LAUNCH_FILE_PATHS)

    robot_to_launch = None
    for file in launch_files:
        fp = file.split(".")
        if len(fp) != 2: continue
        fname, ext = fp
        if ext == "launch":
            name = fname.upper()
            for topic, msg_type in topics:
                names = topic.split("/")
                if name in names[:2]:
                    robot_to_launch = fname

    if robot_to_launch is None:
        robot_to_launch = TEAMBASE_NAME

    launch_file = os.path.join(LAUNCH_FILE_PATHS,robot_to_launch+".launch")
    rospy.loginfo("launching robot file "+ launch_file)
    
    #Launch the 
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    rospy.sleep(2.0)
    launch.start()
    rospy.loginfo("[Launch-Selector] Launch Succesful. All tasks completed.")

    rospy.spin()
    

if __name__ == "__main__":
    main()