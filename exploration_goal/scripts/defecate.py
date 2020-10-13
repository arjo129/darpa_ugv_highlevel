#!/usr/bin/env python
import rospy
from tf.listener import TransformListener
from noroute_mesh.srv import neighbour
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
global rssi_reading
rssi_reading = ""

def last_rssi(message):
    global rssi_reading
    rssi_reading = message.header.frame_id
    print "updating rssi"

def parse_result(message):
    neighbours_rssi_combined = res.response.split("|")
    neighbours = {}
    for n in neighbours_rssi_combined:
        try:
            neighbour, rssi= n.split(",")
            if rssi != "inf":
                neighbours[neighbour] = float(rssi)
        except:
            pass
    return neighbours

def determine_if_use_teambase(message):
    global rssi_reading
    print(rssi_reading, message)
    neighbours_me = parse_result(rssi_reading)
    print(neighbours_me)
    neighbours_team_base = parse_result(message)
    neighbours_me.update(neighbours_team_base)
    return neighbours_me
    
if __name__ == "__main__":
    drop_points = [-55, -60, -65,  -70, -75, -78, -80, -82, -84, -85, -86]
    rospy.init_node("breadcrumb_dropper")
    try:
        robot_name = rospy.get_param("~robot_name")
    except:
        robot_name= "X1"
    print("waiting for comms service")
    rate = rospy.Rate(1)
    rate.sleep()
    dropper = rospy.Publisher(robot_name+"/breadcrumb/deploy", Empty)
    sub =rospy.Subscriber(robot_name+"/comms_publisher", OccupancyGrid, last_rssi)
    
    try:
        get_neighbours  = rospy.ServiceProxy('X1/get_neighbour', neighbour)
        num_of_drops = 0
        while not rospy.is_shutdown():
            res = get_neighbours()
            neighbours = determine_if_use_teambase(res.response)
            print(neighbours)
            if "teambase" in neighbours:
                #print(neighbours["teambase"])
                rssi = neighbours["teambase"]
                if num_of_drops < len(drop_points) and rssi < drop_points[num_of_drops]:
                    num_of_drops += 1
                    dropper.publish(Empty())
                    print ("dropping ", num_of_drops)
            else:
                print ("LOST CONTACT")
            rssi_reading = ""
            rate.sleep()
    except Exception as e:
        print e
    