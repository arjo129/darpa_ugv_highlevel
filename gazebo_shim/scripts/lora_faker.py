#!/usr/bin/python3
"""
Provides a low fidelity simulation of LoRA
"""
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from wireless_msgs.msg import LoraPacket

class LoRASimulator:
    
    def __init__(self):
        self.nodes = ["base_station"]
        self.listeners = {}
        self.senders = {}
        self.gazebo_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_gazebo_update)
    
    def on_gazebo_update(self, msg: ModelStates):
        for name in msg.name:
            if "husky" in name:
                self.nodes.append(name)

    def on_recieve_tx_req(self, msg: LoraPacket):
        # Routing logic (assume perfect routing for now)
        self.senders[msg.to.data].publish(msg)

    def update_ros_pub_subs(self):
        for node in self.nodes:
            if node in self.listeners:
                continue
            
            self.listeners[node] = rospy.Subscriber(node+"/lora/tx", LoraPacket, self.on_recieve_tx_req)
            self.senders[node] = rospy.Publisher(node+"/lora/rx", LoraPacket)

if __name__ == "__main__":
    rospy.init_node("lora_faker")
    lora_sim = LoRASimulator()
    hz = rospy.Rate(1)
    while not rospy.is_shutdown():
        hz.sleep()
        lora_sim.update_ros_pub_subs()