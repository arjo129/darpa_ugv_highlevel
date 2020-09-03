#!/usr/bin/env python
import rospy
from noroute_mesh.srv import neighbour
from std_msgs.msg import Empty

if __name__ == "__main__":
    drop_points = [-55, -60, -65,  -70, -75, -78, -80, -82, -84, -85, -86]
    robot_name = "X1"
    rospy.init_node("breadcrumb_dropper")
    rospy.loginfo("Starting Node")
    rospy.wait_for_service(robot_name+"/get_neighbour")
    rate = rospy.Rate(10)
    dropper = rospy.Publisher(robot_name+"/breadcrumb/deploy", Empty)
    rospy.loginfo("Got comms service")
    try:
        get_neighbours  = rospy.ServiceProxy('X1/get_neighbour', neighbour)
        num_of_drops = 0
        while not rospy.is_shutdown():
            res = get_neighbours()
            rospy.loginfo(res)
            if len(res.response) == 0:
                rate.sleep()
                continue
            neighbours_rssi_combined = res.response.split("|")
            neighbours = {}
            for n in neighbours_rssi_combined:
                neighbour, rssi= n.split(",")
                neighbours[neighbour] = float(rssi) 
            if "teambase" in neighbours:
                #print(neighbours["teambase"])
                rssi = neighbours["teambase"]
                rospy.loginfo("Got RSSI: "+str(rssi))
                if num_of_drops < len(drop_points) and rssi < drop_points[num_of_drops]:
                    num_of_drops += 1
                    dropper.publish(Empty())
                    rospy.loginfo ("dropping %d", num_of_drops)
            else:
                rospy.logerr("LOST CONTACT")

            rate.sleep()
    except Exception as e:
        rospy.logerr(e)
        