#!/usr/bin/env python
import rospy
import tf
from noroute_mesh.srv import neighbour
from std_msgs.msg import Empty


trail = []
last_seen_index = 0

def return_to_base(m):
    for x in trail[last_seen_index:]:
        print x

if __name__ == "__main__":
    drop_points = [-55, -60, -65,  -70, -75, -78, -80, -82, -84, -85, -86]
    rospy.init_node("breadcrumb_dropper")
    rospy.loginfo("Starting Node")
    robot_name = rospy.get_param("~robot_name", "X1")
    rospy.wait_for_service(robot_name+"/get_neighbour")
    rate = rospy.Rate(10)
    dropper = rospy.Publisher(robot_name+"/breadcrumb/deploy", Empty)
    sub = rospy.Subscriber(robot_name+"/return_to_base", Empty, return_to_base)
    rospy.loginfo("Got comms service")
    listener = tf.TransformListener()
    last_sampled = rospy.Time.now()
    
    try:
        get_neighbours  = rospy.ServiceProxy(robot_name+'/get_neighbour', neighbour)
        num_of_drops = 0
        while not rospy.is_shutdown():
            res = get_neighbours()
            rospy.loginfo(res)
            if len(res.response) == 0:
                rate.sleep()
                continue
            try:
                now =rospy.Time.now()
                listener.waitForTransform(robot_name+"/base_link", robot_name+"/world", now, rospy.Duration(1.0))
                translation, rotation = listener.lookupTransform(robot_name+"/base_link", robot_name+"/world", now)
                if now - last_sampled > rospy.Duration(10.0):
                    last_sampled = now
                    trail.append(translation)
                    rospy.loginfo("Recorded ")
            except tf.Exception as e:
                rospy.logerr(e.msg)

            neighbours_rssi_combined = res.response.split("|")
            neighbours = {}
            for n in neighbours_rssi_combined:
                neighbour, rssi= n.split(",")
                neighbours[neighbour] = float(rssi) 
            if "teambase" in neighbours:
                #Update the trail to state that this should be 
                last_seen_index = len(trail) - 1 if len(trail) > 0 else 0
                rssi = neighbours["teambase"]
                rospy.loginfo("Got RSSI: "+str(rssi))
                if num_of_drops < len(drop_points) and rssi < drop_points[num_of_drops]:
                    num_of_drops += 1
                    dropper.publish(Empty())
                    rospy.loginfo ("dropping %d", num_of_drops)
            else:
                rospy.logerr("LOST CONTACT WITH BASE")
            rate.sleep()
    except Exception as e:
        rospy.logerr(e)
        