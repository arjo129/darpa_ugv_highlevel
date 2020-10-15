#!/usr/bin/env python
import rospy
import json
from noroute_mesh.srv import neighbour, send_map
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from tf.listener import TransformListener

rospy.init_node("u1start")
rospy.loginfo("waiting for comms service")
rospy.wait_for_service("/U1/send_map")
rospy.loginfo("Waiting for start time")

send_map = rospy.ServiceProxy("/U1/send_map", send_map)
get_neighbour = rospy.ServiceProxy("/U1/get_neighbour", neighbour)
pub = rospy.Publisher("/U1/waypoints", Path)
start_exploring = rospy.Publisher("/U1/start_exploration", Empty)
global first
first = True
global listener
listener = TransformListener()

def create_point_stamped(point, _time, frame):
    point_stamped = PointStamped()
    point_stamped.header.frame_id = frame
    point_stamped.header.stamp = _time
    point_stamped.point.x = point[0]
    point_stamped.point.y = point[1]
    point_stamped.point.z = point[2]
    return point_stamped


def create_pose_stamped(point, _time, frame):
    point_stamped = PoseStamped()
    point_stamped.header.frame_id = frame
    point_stamped.header.stamp = _time
    point_stamped.pose.position.x = point[0]
    point_stamped.pose.position.y = point[1]
    point_stamped.pose.position.z = point[2]
    return point_stamped


def on_recieve(msg):
    global first, listener
    message = msg.header.frame_id
    data = json.loads(message)
    if data["type"] != "trail":
        rospy.logerr("unknown message type skipping")
    trail = data["trail_to_follow"]
    points = transform_points_from_artifact(listener, trail)
    my_path = Path()
    for point in points:
        pose = create_pose_stamped(point, rospy.Time.now(), "U1/world")
        pose.point.z += 1.0
        my_path.poses.append(pose)
    
    if len(my_path.poses) >1 and first:
        rospy.loginfo("Recieved the path information")
        first = False
        pub.publish(my_path)
        start_exploration()
    else:
        rospy.loginfo("Triggered exploration already")
    

def transform_points_from_artifact(listener, points):
    now = rospy.Time.now()
    for_conversion = []
    for point in points:
        for_conversion.append(create_point_stamped(point, now,  "artifact_origin"))
    res = []
    try:
        listener.waitForTransform("U1/world", "artifact_origin", now, rospy.Duration(3))
        
        for to_convert in for_conversion:
            r = listener.transformPoint("U1/world", to_convert)
            pt = [r.point.x, r.point.y, r.point.z]
            res.append(pt)
    except Exception as e:
        rospy.logerr(e)
    return res

def request_init_path():
    message = {}
    message["type"] = "query"
    message["robot"] = "U1"
    packet = OccupancyGrid()
    packet.header.frame_id = json.dumps(message)
    send_map("teambase", packet)
    rospy.loginfo("saent request")

def start_exploration():
    mes = Empty()
    start_exploring.publish(mes)

sub = rospy.Subscriber("/U1/comms_publisher", OccupancyGrid, on_recieve)
rospy.sleep(7) #wait for drone to be airborne
while not rospy.is_shutdown():
    if first:
        request_init_path()
    else:
        start_exploration()
    rospy.sleep(1.0)
