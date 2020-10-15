#!/usr/bin/env python
import rospy
from tf.listener import TransformListener
from noroute_mesh.srv import neighbour, send_map
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseArray
import json
global rssi_reading

DROP_BEACON = 44

rssi_reading = ""
global positions
positions = [[0,0,0]]

last_position = [0,0,0]

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

def serialize_telemetry(robot_name, position, beacons):
    global positions
    message = {}
    message["type"] = "telemetry"
    message["robot"] = robot_name
    message["beacons"] = beacons
    message["position"] = last_position
    packet = OccupancyGrid()
    packet.header.frame_id = json.dumps(message)
    return packet

def create_point_stamped(point, _time, frame):
    point_stamped = PointStamped()
    point_stamped.header.frame_id = frame
    point_stamped.header.stamp = _time
    point_stamped.point.x = point[0]
    point_stamped.point.y = point[1]
    point_stamped.point.z = point[2]
    return point_stamped

def transform_points_to_artifact(listener, points, frame):
    now = rospy.Time.now()
    for_conversion = []
    for point in points:
        for_conversion.append(create_point_stamped(point, now, frame))
    res = []
    try:
        listener.waitForTransform("artifact_origin", frame, now, rospy.Duration(3))
        
        for to_convert in for_conversion:
            r = listener.transformPoint("artifact_origin", to_convert)
            pt = [r.point.x, r.point.y, r.point.z]
            res.append(pt)
    except Exception as e:
        rospy.logerr(e)
    return res

def breadcrumbs_recieved(msg):
    global positions
    rospy.loginfo("Recieived breadcrumb update!!")
    for pose in msg.poses:
        pt = [0,0,0]
        pt[0] = pose.position.x
        pt[1] = pose.position.y
        pt[2] = pose.position.z
        positions.append(pt)
    rospy.loginfo("Finished breadcrumb update!!")

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
    send_map = rospy.ServiceProxy("send_map", send_map)
    subscriber = rospy.Subscriber("breadcrumb_list", PoseArray, breadcrumbs_recieved)
    listener = TransformListener()
    while not rospy.is_shutdown():
        print (positions)
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
        
        if euclidean_distance(last_position, trans) > 5:
            rospy.loginfo("Sending data")
            last_position = trans
            beacons = transform_points_to_artifact(listener, positions, robot_name+"/world")
            last_pos = transform_points_to_artifact(listener, [last_position], robot_name+"/world")
            tele = serialize_telemetry(robot_name, last_position[0], beacons)
            send_map("teambase", tele)