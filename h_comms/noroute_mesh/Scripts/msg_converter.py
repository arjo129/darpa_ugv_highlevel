#!/usr/bin/env python3
import rospy
import json
from rospy_message_converter import json_message_converter
from rospy_message_converter import message_converter

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

from noroute_mesh.srv import string_to_map, string_to_mapResponse
from noroute_mesh.srv import map_to_string, map_to_stringResponse

'''
A character will be appended in front of the payload to indicate the msg_type of the ros message.
nav_msgs/OccupancyGrid      1
'''

def map2string(req):
    message = req.grid
    json_str = json_message_converter.convert_ros_message_to_json(message)
    r = String()
    print(json.dumps(json_str))
    r.data = "1" + json.dumps(json_str)
    print("Converting Map to String")
    return r

def string2map(req):
    d = json.loads(req.str.data[1:])
    print(d)
    message =  json_message_converter.convert_json_to_ros_message('nav_msgs/OccupancyGrid', d)
    res = string_to_mapResponse()
    res.grid = message
    print("Converting String to Map")
    return res

if __name__ == "__main__":
    rospy.init_node('ros_message_converter')
    m = rospy.Service('map_to_string', map_to_string, map2string)
    s = rospy.Service('string_to_map', string_to_map, string2map)
    print("READY")
    rospy.spin()