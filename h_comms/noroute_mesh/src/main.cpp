#include <unordered_set>
#include "iostream"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "Node.hpp"
#include "Eth.hpp"

// messages
#include "std_msgs/String.h"
#include <ignition/msgs.hh>
#include "noroute_mesh/neighb.h"
#include "noroute_mesh/neighbArray.h"
#include "subt_ign/protobuf/artifact.pb.h"
#include "subt_ign/CommonTypes.hh"

// services
#include "noroute_mesh/send_map.h"
#include "noroute_mesh/neighbour.h"
#include "noroute_mesh/map_to_string.h"
#include "noroute_mesh/string_to_map.h"
#include "noroute_mesh/send_artifact.h"

#include "nav_msgs/OccupancyGrid.h"
#include "subt_communication_broker/subt_communication_client.h"

class nomesh {
private:
    std::string name;
    std::unordered_set<uint32_t> sequences;
    aodv::Node *node;
    subt::CommsClient *cc;
    uint32_t prevSeq = 0;
    
public:
    uint32_t id;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer map_service, neighbour_service, discovery_service, artifact_service;
    ros::ServiceClient map_to_string_client, string_to_map_client;

    nomesh(std::string robotName, uint32_t robotId) {
        this->name = robotName;
        this->id = robotId;
        cc = new subt::CommsClient(nomesh::name);
        node = new aodv::Node(robotId, nomesh::name, subt::communication_broker::kBroadcast);
        cc->StartBeaconInterval(ros::Duration(PING_INTERVAL));
        start_server();
    }

    ~nomesh(){
        delete cc;
        delete node;
    }

    void start_server(){
        ROS_INFO("Name: %s, ID: %u", this->name.c_str(), this->id);

        map_service = nh.advertiseService(nomesh::name + "/send_map",&nomesh::sendMap,this);
        neighbour_service = nh.advertiseService(nomesh::name + "/get_neighbour",&nomesh::getNeighbour,this);
        artifact_service = nh.advertiseService(nomesh::name + "/send_artifact",&nomesh::sendArtifact,this);
        ROS_INFO("Send map and neighbour service advertised");

        map_to_string_client = nh.serviceClient<noroute_mesh::map_to_string>("map_to_string");
        string_to_map_client = nh.serviceClient<noroute_mesh::string_to_map>("string_to_map");
        ROS_INFO("ROS messages conversion service subscribed");

        pub = nh.advertise<nav_msgs::OccupancyGrid>(nomesh::name + "/comms_publisher",1);
        nomesh::cc->Bind(std::bind(&nomesh::handlePacket,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), this->name, 4100);
    }
    
    void handlePacket(const std::string &_srcAddress, const std::string &_dstAddress, const uint32_t _dstPort, const std::string &_data){
        // Handle artifact score acknowledgements
        subt::msgs::ArtifactScore res;
        if (res.ParseFromString(_data))
        {
            ROS_INFO("Message from [%s] to [%s] on port [%u]:\n [%s]", _srcAddress.c_str(),_dstAddress.c_str(), _dstPort, res.DebugString().c_str());
        }
        else {
            // Deserialise the received data
            tl::optional<aodv::Eth> e = nomesh::node->receive(_data, cc);

            if (e && sequences.count(e.value().seq) == 0) // Prevent data from being published twice.
            {
                aodv::Eth eth = e.value();

                if (eth.seq < prevSeq)
                {
                    sequences.clear(); // Sequence number has overflowed, reset previous trackings.
                }
                prevSeq = eth.seq;
                sequences.insert(eth.seq);

                ROS_INFO("Received map from source %s", eth.src.c_str());
                if (eth.payload.at(0) == '1'){
                    std::cout << "Received nav_msgs::OccupancyGrid message type." << std::endl;
                    noroute_mesh::string_to_map srv;
                    srv.request.str.data = eth.payload;
                    if (string_to_map_client.call(srv))
                    {
                        pub.publish(srv.response.grid);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service: string_to_map.");
                    }
                }
                else{
                    ROS_ERROR("Unable to retrieve ros_msg type.");
                }
            }
        }
    }

    bool sendMap(noroute_mesh::send_map::Request &req, noroute_mesh::send_map::Response &res){
        aodv::Eth e;
        
        e.dstLength = req.dest.length();
        e.dst = req.dest;
        e.srcLength = nomesh::name.length();
        e.src = nomesh::name;

        noroute_mesh::map_to_string srv;
        srv.request.grid = req.grid;
        if (map_to_string_client.call(srv))
        {
            e.payload = srv.response.str.data;
            e.payloadLength = e.payload.size();
            nomesh::node->send(e, this->cc, true);
            ROS_INFO("Sent serialised payload of size %u bytes", e.payloadLength);
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service: map_to_string");
            return false;
        }
    }

    bool getNeighbour(noroute_mesh::neighbour::Request &req, noroute_mesh::neighbour::Response &res){
        int num = 0;
        subt::CommsClient::Neighbor_M neigh = cc->Neighbors();
        std::stringstream ss;

        // Get current simulation time to compare with.
        double currentTime = ros::Time::now().toSec();

        // Filter out neighbours that are outdated.
        ROS_INFO("Requested for neighbours information");
        for (subt::CommsClient::Neighbor_M::iterator it = neigh.begin(); it != neigh.end(); it++) {
            if (it->first == this->name)
            {
                continue; // ignore ourselves.
            }
            
            double neighTime = it->second.first;
            if (currentTime - neighTime <= NEIGHBOUR_TIME_THRESH)
            {
                if (num > 0)
                {
                    ss << "|"; // delimiter
                }
                ss << it->first << "," << it->second.second; // print Name and RSSI
                ROS_INFO("Name: %s, RSSI: %lf", it->first.c_str(), it->second.second);
                num++;
            }
        }
        res.num_neighbours = num;
        res.response = ss.str();
        return true;
    }

    bool sendArtifact(noroute_mesh::send_artifact::Request &req, noroute_mesh::send_artifact::Response &res){
        ignition::msgs::Pose pose;
        pose.mutable_position()->set_x(req.x);
        pose.mutable_position()->set_y(req.y);
        pose.mutable_position()->set_z(req.z);

        // Fill the type and pose.
        subt::msgs::Artifact artifact;
        if (req.type == "TYPE_BACKPACK"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_BACKPACK));
        }
        else if (req.type == "TYPE_DUCT"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_DUCT));
        }
        else if (req.type == "TYPE_DRILL"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_DRILL));
        }
        else if (req.type == "TYPE_ELECTRICAL_BOX"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_ELECTRICAL_BOX));
        }
        else if (req.type == "TYPE_EXTINGUISHER"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_EXTINGUISHER));
        }
        else if (req.type == "TYPE_PHONE"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_PHONE));
        }
        else if (req.type == "TYPE_RADIO"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_RADIO));
        }
        else if (req.type == "TYPE_RESCUE_RANDY"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_RESCUE_RANDY));
        }
        else if (req.type == "TYPE_TOOLBOX"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_TOOLBOX));
        }
        else if (req.type == "TYPE_VALVE"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_VALVE));
        }
        else if (req.type == "TYPE_VENT"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_VENT));
        }
        else if (req.type == "TYPE_GAS"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_GAS));
        }
        else if (req.type == "TYPE_HELMET"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_HELMET));
        }
        else if (req.type == "TYPE_ROPE"){
            artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_ROPE));
        }
        else {
            std::cout << "ReportArtifact(): Not a valid artifact type" << std::endl;
            res.response = false;
            return false;
        }
        artifact.mutable_pose()->CopyFrom(pose);
        // Serialize the artifact.
        std::string serializedData;
        if (!artifact.SerializeToString(&serializedData)) {
            std::cout << "ReportArtifact(): Error serializing message\n"
                    << artifact.DebugString() << std::endl;
        }
    
        // Report it.
        cc->SendTo(serializedData, subt::kBaseStationName);
        ROS_INFO("Artifact Reported.");
        res.response = true;
        return true;
    }

private:
    const double NEIGHBOUR_TIME_THRESH = 3.0;
    const double PING_INTERVAL = 1.0;
};

int main(int argc, char** argv)
{
    if (argc != 5){
        printf("Enter robot name and id as the arguments for this CommsClient.\n");
        printf("Eg: [rosrun ...] RobotA 1\n");
        return 0;
    }
    std::string robotName = argv[1];
    std::string robotIdStr = argv[2];
    uint32_t robotId = stoi(robotIdStr);
    ros::init(argc, argv, robotName + "_nomesh_API", ros::init_options::AnonymousName);
    nomesh robot(robotName, robotId);
    ros::spin();
}
