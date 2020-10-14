#!/usr/bin/env python
import rospy
import json
from noroute_mesh.srv import neighbour, send_map
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid

rospy.init_node("teambase_map_sender")
print("waiting for comms service")
rospy.wait_for_service("/teambase/send_map")
send_map = rospy.ServiceProxy("/teambase/send_map", send_map)
get_neighbour = rospy.ServiceProxy("/teambase/get_neighbour", neighbour)

global robot_trails
robot_trails = {}
global breadcrumbs
breadcrumbs = []


def callback(message):
    global robot_trails
    global breadcrumbs
    rospy.loginfo("recieved message")
    msg = message.header.frame_id
    try:
        message = json.loads(msg)
        print(message)
        if message["type"] == "telemetry" and message["robot"] in robot_trails:
            robot_trails[message["robot"]].append(message["position"])
        elif message["type"] == "telemetry" :
            robot_trails[message["robot"]] = [message["position"]]
        elif message["type"] == "query":
            send_robot_trail(message["robot"])
    except Exception:
        rospy.logerr("Malformatted JSON")

def send_robot_trail(robot):
    msg = OccupancyGrid()
    prev_max = 0
    idx = ""
    
    for r in robot_trails:
        if len(robot_trails[r]) >= prev_max:
            prev_max =  len(robot_trails[r])
            idx = r
        
    if idx == "":
        print "no robots registered yet"
        message = {}
        message["type"] = "trail"
        message["trail_to_follow"] = []
        message["breadcrumbs"] = breadcrumbs  
        msg.header.frame_id = json.dumps(message)
        send_map(robot, msg)
        return
    
    message = {}
    message["type"] = "trail"
    message["trail_to_follow"] = robot_trails[idx]
    message["breadcrumbs"] = breadcrumbs  
    msg.header.frame_id = json.dumps(message)
    send_map(robot, msg)


sub = rospy.Subscriber("/teambase/comms_publisher", OccupancyGrid, callback)
rospy.spin()

