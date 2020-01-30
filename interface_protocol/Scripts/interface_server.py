#!/usr/bin/env python3

import rospy
import requests
import pprint
import json

from interface_protocol.srv import PostReport,PostReportResponse
from interface_protocol.srv import GetStatus,GetStatusResponse
from interface_protocol.srv import MappingUpdate,MappingUpdateResponse

from std_msgs.msg import String
from rospy_message_converter import json_message_converter
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray

class score_interface_server:
    base_url = "http://localhost:8000"
    token_file = ""
    token = ""

    def __init__(self):
        self.get_token()
        srv_post = rospy.Service('post_report', PostReport, self.post_report)
        srv_status = rospy.Service('get_status', GetStatus, self.get_status)
        print("[scoring_interface] STATUS: READY")

    def get_token(self):
        infile = open(self.token_file)
        self.token = infile.read()[:-1]
        #print(self.token)
        infile.close()

    def post_report(self,req):
        data = {'x' : req.x, 'y' : req.y, 'z' : req.z, 'type': req.type}
        head = {"Authorization":"bearer " + self.token}
        url = self.base_url + '/api/artifact_reports/'

        response = requests.post(url, json=data, headers=head)
        print("[POST][post_report] RESPONSE: " + str(response))
        j = response.json()
        pprint.pprint(j)

        temp = str(j)
        temp = temp.replace("\'","\"")
        j = json.loads(temp)
        
        resp = PostReportResponse()
        resp.url = str(j["url"])
        resp.id = int(j["id"])
        resp.x = float(j["x"])
        resp.y = float(j["y"])
        resp.z = float(j["z"])
        resp.type = str(j["type"])
        resp.type = str(j["submitted_datetime"])
        resp.run_clock = float(j["run_clock"])
        resp.team = str(j["team"])
        resp.run = str(j["run"])
        resp.report_status = str(j["report_status"])
        resp.score_change = int(j["score_change"])

        return resp

    def get_status(self,req):
        head = {"Authorization":"bearer " + self.token}
        url = self.base_url + '/api/status/'

        response = requests.get(url, headers=head)
        print("[GET][get_status] RESPONSE: " + str(response))
        j = response.json()
        pprint.pprint(j)

        temp = str(j)
        temp = temp.replace("\'","\"")
        j = json.loads(temp)

        resp = GetStatusResponse()
        resp.current_team = str(j["current_team"])
        resp.remaining_reports = int(j["remaining_reports"])
        resp.run_clock = float(j["run_clock"])
        resp.score = int(j["score"])

        return resp

class mapping_interface_server():
    base_url = "http://localhost:8000"
    token_file = ""
    token = ""

    grid = OccupancyGrid()
    pose = PoseArray()
    mark = MarkerArray()

    def __init__(self):
        self.get_token()
        sub_grid = rospy.Subscriber("/map", OccupancyGrid, self.update_grid)
        sub_pose = rospy.Subscriber("/pose", PoseArray, self.update_pose)
        sub_mark = rospy.Subscriber("/mark", MarkerArray, self.update_mark)

        srv_grid = rospy.Service('post_map_update', MappingUpdate, self.post_map_update)
        srv_pose = rospy.Service('post_telemetry_update', MappingUpdate, self.post_telemetry_update)
        srv_mark = rospy.Service('post_markers_update', MappingUpdate, self.post_markers_update)

        print("[mapping_interface] STATUS: READY")

    def get_token(self):
        infile = open(self.token_file)
        self.token = infile.read()[:-1]
        #print(self.token)
        infile.close()

    def post_map_update(self,req):
        json_str = json_message_converter.convert_ros_message_to_json(self.grid)
        message = json.loads(json_str)

        data = {'type': 'OccupancyGrid', 'msg': message}
        head = {"Authorization":"bearer " + self.token}
        url = self.base_url + '/map/update/'

        response = requests.post(url, json=data, headers=head)
        print("[POST][map_update] RESPONSE: " + str(response))
        return MappingUpdateResponse(str(response))

    def post_telemetry_update(self,req):
        json_str = json_message_converter.convert_ros_message_to_json(self.pose)
        message = json.loads(json_str)
        
        data = message
        head = {"Authorization":"bearer " + self.token}
        url = self.base_url + '/state/update/'

        response = requests.post(url, json=data, headers=head)
        print("[POST][telemetry_update] RESPONSE: " + str(response))
        return MappingUpdateResponse(str(response))

    def post_markers_update(self,req):
        json_str = json_message_converter.convert_ros_message_to_json(self.mark)
        message = json.loads(json_str)
        
        data = message
        head = {"Authorization":"bearer " + self.token}
        url = self.base_url + '/markers/update/'

        response = requests.post(url, json=data, headers=head)
        print("[POST][markers_update] RESPONSE: " + str(response))
        return MappingUpdateResponse(str(response))

    # ROS Updates
    def update_grid(self,msg):
        self.grid = msg.data
    def update_pose(self,msg):
        self.pose = msg.data
    def update_mark(self,msg):
        self.mark = msg.data

if __name__ == "__main__":
    try:
        rospy.init_node('interface_protocol_server')
        s = score_interface_server()
        m = mapping_interface_server()
        rospy.spin()

    except Exception as e:
        print ("Protocol Interface Server failed: %s" % e)