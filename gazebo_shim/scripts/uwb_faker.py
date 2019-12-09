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
        self.noise_provider = np.random.normal(0, 0.4)
    
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
            for other in self.uwb_poses:
                
                if other == name:
                    continue

                print(self.get_distance(name, other))

    def get_distance(self, this, other):
        this_pos = self.uwb_poses[this].position
        other_pos = self.uwb_poses[other].position
        this_pos = np.array([this_pos.x, this_pos.y, this_pos.z])
        other_pos = np.array([other_pos.x, other_pos.y, other_pos.z])
        return np.linalg.norm(other_pos - this_pos)
        

if __name__ == "__main__":
    rospy.init_node("UWB_simulator")
    uwb = UWBSimulationShim()
    hz = rospy.Rate(10)
    while not rospy.is_shutdown():
        hz.sleep()
        uwb.publish_results()