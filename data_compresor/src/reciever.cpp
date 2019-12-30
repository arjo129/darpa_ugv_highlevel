#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <data_compressor/devices/laserscan.h>
#include <geometry_msgs/TransformStamped.h>
#include <wireless_msgs/LoraPacket.h>

class Reciever {
    ros::NodeHandle nh;
    ros::Subscriber sub;
    std::unordered_map<std::string, data_compressor::PacketRecieveQueue> recvQueues; 
    std::unordered_map<std::string, std::unordered_map<data_compressor::MessageType, ros::Publisher> > publishers;
    data_compressor::LaserScanCompressor lscan;
    std::string nameSpace;
    tf::TransformBroadcaster tfBroadcaster;

    void routeDecompression(std::string sender, data_compressor::DataPacket packet){
        geometry_msgs::TransformStamped tstamped;
        tstamped.transform.translation.x = packet.estimated_pose.position.x;
        tstamped.transform.translation.y = packet.estimated_pose.position.y;
        tstamped.transform.translation.z = packet.estimated_pose.position.z;
        tstamped.transform.rotation = packet.estimated_pose.orientation;
        tstamped.header.frame_id = sender+"/odom";
        tstamped.child_frame_id = sender+"/base_link";
        tfBroadcaster.sendTransform(tstamped);
        switch(packet.packet_type) {
            case data_compressor::MessageType::LASER_SCAN:
                sensor_msgs::LaserScan scan = lscan.decompress(packet);
                scan.header.frame_id = sender+"/laser";
                if(publishers.count(sender) == 0){
                    publishers[sender] = std::unordered_map<data_compressor::MessageType, ros::Publisher>();
                }

                if(publishers[sender].count(data_compressor::MessageType::LASER_SCAN) == 0) {
                    publishers[sender][data_compressor::MessageType::LASER_SCAN] = nh.advertise<sensor_msgs::LaserScan>(nameSpace+"/"+sender+"/scan", 10); 
                }
                publishers[sender][data_compressor::MessageType::LASER_SCAN].publish(scan);      
                break;
        }
    }

public:
    Reciever(ros::NodeHandle _nh, std::string _namespace): nh(_nh) {
        sub = nh.subscribe(_namespace+"/rx", 10, &Reciever::onLoraPacketRecieved, this);
        this->nameSpace = _namespace;
    }

    void onLoraPacketRecieved(wireless_msgs::LoraPacket packet) {
        this->recvQueues[packet.from.data].enqueuePacket(data_compressor::toPhysicalChunk(packet), ros::Time::now().sec); 
    }

    void publish(){
        for(auto queue: this->recvQueues){
            std::string sender = queue.first;
            std::vector<data_compressor::ByteStream> bs = queue.second.getBuffer();
            for(int i = 0; i < bs.size(); i++){
                data_compressor::DataPacket packet = fromByteStream(bs[i]);
                routeDecompression(sender, packet);
            }
            
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "basestation_relay");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    Reciever recv(nh, "base_station");
    while(ros::ok()){
        ros::spinOnce();
        recv.publish();
        rate.sleep();
    }
    return 0;
}