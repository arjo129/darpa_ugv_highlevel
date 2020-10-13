#!/usr/bin/env python
import rospy
from tf.listener import TransformListener
from noroute_mesh.srv import neighbour
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
global rssi_reading

DROP_BEACON = 38

rssi_reading = ""
global positions
positions = [[0,0,0]]

def euclidean_distance(d1, d2):
    dist = 0
    for i in range(len(d1)):
        r = d1[i] - d2[i]
        dist += r*r
    return dist**0.5

def get_closest(position):
    global positions
    min_dist = 9999999
    for p in positions:
        d = euclidean_distance(p, position) 
        if d < min_dist:
            min_dist =d
    return min_dist

if __name__ == "__main__":
    rospy.init_node("breadcrumb_dropper")
    try:
        robot_name = rospy.get_param("~robot_name")
    except:
        robot_name= "X1"
    print("waiting for comms service")
    rate = rospy.Rate(1)
    rate.sleep()
    dropper = rospy.Publisher("breadcrumb/deploy", Empty)
    listener = TransformListener()
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform(robot_name+"/world", robot_name+"/base_link", now, rospy.Duration(3))
            trans, rot = listener.lookupTransform(robot_name+"/world", robot_name+"/base_link" , now)
        except Exception as e:
            print (e)
            continue

        rospy.sleep(1.0)  
        rospy.loginfo (trans)
        res = get_closest(trans)
        rospy.loginfo ("distance from nearest breadcrumb %f", res)
        if res > DROP_BEACON:
            empt = Empty()
            dropper.publish(empt)
            positions.append(trans)
            rospy.loginfo("Deploying breadcrumb")
