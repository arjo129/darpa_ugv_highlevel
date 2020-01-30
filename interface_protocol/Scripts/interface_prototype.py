#!/usr/bin/env python3
import rospy
import requests
import json
import pprint
from rospy_message_converter import json_message_converter
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray

base_url = "http://localhost:8000"
token = "tokentokentoken1"

# retrieve the status of the run
def get_status():
    response=requests.get(base_url + "/api/status/",headers={"Authorization":"bearer " + token})
    j = response.json()
    print("[GET][get_status] RESPONSE:")
    pprint.pprint(j)

# send report of the artifacts to the command post
# there is a maximum number of reports that can be done to the command post
def post_report(_x, _y, _z, _type):
    data = {'x' : _x, 'y' : _y, 'z' : _z, 'type': _type}
    head = {"Authorization":"bearer " + token}
    url = base_url + '/api/artifact_reports/'

    response = requests.post(url, json=data, headers=head)
    j = response.json()
    print("[POST][post_report] RESPONSE:")
    pprint.pprint(j)

# send map updates to the command post
# essentially sending a nav_msgs/OccupancyGrid message
def post_map_update(msg):
    json_str = json_message_converter.convert_ros_message_to_json(msg)
    message = json.loads(json_str)

    data = {'type': 'OccupancyGrid', 'msg': message}
    head = {"Authorization":"bearer " + token}
    url = base_url + '/map/update/'

    response = requests.post(url, json=data, headers=head)
    print("[POST][map_update] RESPONSE: " + str(response))

# send telemetry updates to the command post
# essentially sending a geometry_msgs/PoseArray message
def post_telemetry_update(msg):
    json_str = json_message_converter.convert_ros_message_to_json(msg)
    message = json.loads(json_str)
    
    data = message
    head = {"Authorization":"bearer " + token}
    url = base_url + '/state/update/'

    response = requests.post(url, json=data, headers=head)
    print("[POST][telemetry_update] RESPONSE: " + str(response))

# send markers to the command post
# essentially sending a visualization_msgs/MarkerArray message
def post_markers_update(msg):
    json_str = json_message_converter.convert_ros_message_to_json(msg)
    message = json.loads(json_str)
    
    data = message
    head = {"Authorization":"bearer " + token}
    url = base_url + '/markers/update/'

    response = requests.post(url, json=data, headers=head)
    print("[POST][markers_update] RESPONSE: " + str(response))

#get_status()

post_report(321,5432,432,"backpack")

m = OccupancyGrid()
p = PoseArray()
ma = MarkerArray()

#post_map_update(m)
#post_telemetry_update(p)
#post_markers_update(ma)