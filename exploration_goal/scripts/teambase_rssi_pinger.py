#!/usr/bin/env python
import rospy
from noroute_mesh.srv import neighbour, send_map
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid

rospy.init_node("teambase_map_sender")
print("waiting for comms service")
rospy.wait_for_service("teambase/send_map")
send_map = rospy.ServiceProxy("teambase/send_map", send_map)
get_neighbour = rospy.ServiceProxy("teambase/get_neighbour", neighbour)

while not rospy.is_shutdown():
    occ = OccupancyGrid();
    neighs = get_neighbour()
    occ.header.frame_id = neighs.response
    send_map("X1", occ)
    rospy.sleep(0.5)
    rospy.loginfo("Sending data")