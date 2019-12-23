#!/usr/bin/python3
"""
Provides a low fidelity simulation of UWB messages 
"""
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from wireless_msgs.msg import uwb

class UWBSimulationShim:

    def __init__(self):
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_gazebo_update)
        self.uwb_poses = {}
        self.publishers = {}
    
    def on_gazebo_update(self, msg: ModelStates):
        for index, name in enumerate(msg.name):
            if "husky" in name:
                self.uwb_poses[name] = msg.pose[index]

    def fetch_publishers(self):
        for node in self.uwb_poses:
            if node in self.publishers:
                continue
            self.publishers[node] = rospy.Publisher(node+"/uwb_rangers", uwb)
    
    def publish_results(self):
        
        self.fetch_publishers()

        for name in self.uwb_poses:
            msg = uwb()
            msg.header.frame_id = name+"/base_link"
            msg.name.data = "home_beacon"
            msg.distance.data = self.get_distance_from_home(name) + np.random.normal(0, 0.4)
            self.publishers[name].publish(msg)
            for other in self.uwb_poses:
                
                if other == name:
                    continue

                msg = uwb()
                msg.header.frame_id = name+"/base_link"
                msg.header.stamp = rospy.Time()
                msg.name.data = other
                msg.distance.data = self.get_distance(name, other) + np.random.normal(0, 0.4)
                self.publishers[name].publish(msg)

    def get_distance(self, this, other):
        this_pos = self.uwb_poses[this].position
        other_pos = self.uwb_poses[other].position
        this_pos = np.array([this_pos.x, this_pos.y, this_pos.z])
        other_pos = np.array([other_pos.x, other_pos.y, other_pos.z])
        return np.linalg.norm(other_pos - this_pos)

    def get_distance_from_home(self, this):
        this_pos = self.uwb_poses[this].position
        this_pos = np.array([this_pos.x, this_pos.y, this_pos.z])
        return np.linalg.norm(this_pos)
        

if __name__ == "__main__":
    rospy.init_node("UWB_simulator")
    simulation = UWBSimulationShim()
    hz = rospy.Rate(10)
    while not rospy.is_shutdown():
        hz.sleep()
        simulation.publish_results()