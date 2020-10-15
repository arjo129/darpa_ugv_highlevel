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
robot_trails = {}#{'X1': [[5.2004780769348145, -2.444396734237671, -0.0180668905377388], [10.032371741074781, -3.7997345190781813, 0.03697027359157801], [15.765952550447905, -4.439849926875188, 0.25452865087069], [21.168655395507812, -3.8392802079518638, 0.4649305840333303], [26.78882598876953, -3.2424774169921875, 0.7068935632705688], [32.10504150390625, -2.352442979812622, 0.6796616911888123]]}
global breadcrumbs
breadcrumbs = [[0, 0, 0]]

def euclidean_distance(d1, d2):
    dist = 0
    for i in range(len(d1)):
        r = d1[i] - d2[i]
        dist += r*r
    return dist**0.5

def get_closest(position):
    global breadcrumbs
    min_dist = 9999999
    for p in breadcrumbs:
        d = euclidean_distance(p, position) 
        if d < min_dist:
            min_dist =d
    return min_dist

def add_breadcrumbs(crumbs):
    global breadcrumbs
    for crumb in crumbs:
        d = get_closest(crumb)
        if d < 20:
            continue
        breadcrumbs.append(crumb)

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
            rospy.loginfo ("trails")
            rospy.loginfo (robot_trails)
            rospy.loginfo ("breadcrumbs")
            rospy.loginfo (breadcrumbs)
            add_breadcrumbs(message["beacons"])
        elif message["type"] == "telemetry" :
            robot_trails[message["robot"]] = [message["position"]]
        elif message["type"] == "query":
            send_robot_trail(message["robot"])
    except Exception as e:
        rospy.logerr("Malformatted JSON")
        print(e)

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

