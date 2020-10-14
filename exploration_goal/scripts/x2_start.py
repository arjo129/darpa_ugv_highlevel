#!/usr/bin/env python
import rospy
import json
from noroute_mesh.srv import neighbour, send_map
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from tf.listener import TransformListener

rospy.init_node("x2start")
print("waiting for comms service")
rospy.wait_for_service("/X2/send_map")
send_map = rospy.ServiceProxy("/X2/send_map", send_map)
get_neighbour = rospy.ServiceProxy("/X2/get_neighbour", neighbour)
pub = rospy.Publisher("/X2/headstart_waypoint", Path)

def create_point_stamped(point, _time, frame):
    point_stamped = PointStamped()
    point_stamped.header.frame_id = frame
    point_stamped.header.stamp = _time
    point_stamped.point.x = point[0]
    point_stamped.point.y = point[1]
    point_stamped.point.z = point[2]
    return point_stamped


def on_recieve(msg):
    message = msg.header.frame
    data = json.loads(msg)
    if data["type"] != "trail":
        rospy.logerr("unknown message type skipping")
    trail = data["trail_to_follow"]
    points = transform_points_from_artifact(listener, trail)
    my_path = Path()
    for point in points:
        pose = create_point_stamped(point, rospy.Time.now(), "X2/world")
        my_path.poses.append(pose)
    pub.publish(my_path)
    

def transform_points_from_artifact(listener, points):
    now = rospy.Time.now()
    for_conversion = []
    for point in points:
        for_conversion.append(create_point_stamped(point, now,  "artifact_origin"))
    res = []
    try:
        listener.waitForTransform("X2/world", "artifact_origin", now, rospy.Duration(3))
        
        for to_convert in for_conversion:
            r = listener.transformPoint("X2/world", to_convert)
            pt = [r.point.x, r.point.y, r.point.z]
            res.append(pt)
    except Exception as e:
        rospy.logerr(e)
    return res

def request_init_path():
    message = {}
    message["type"] = "query"
    message["robot"] = "X2"
    packet = OccupancyGrid()
    packet.header.frame_id = json.dumps(message)
    send_map("teambase", packet)
    print("saent request")

sub = rospy.Subscriber("/X2/comms_publisher", OccupancyGrid, on_recieve)

while not rospy.is_shutdown():
    request_init_path()
    rospy.sleep(1.0)
