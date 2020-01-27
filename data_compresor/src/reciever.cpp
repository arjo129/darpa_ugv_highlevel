/***
 * Decodes packets coming in via LoRA and sends heartbeat status signal to all robots
 * 
 */  
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <wireless_msgs/LoraPacket.h> 
#include <data_compressor/zip.h>
#include <data_compressor/protocol.h>
#include <data_compressor/msgs/laserscan.h>

ros::Publisher pub;
ros::Subscriber loraSub;
void handleLaserScan(std::string from, std::vector<uint8_t> data) {
    AdaptiveTelemetryScan scan = decodeScan(data);
    sensor_msgs::LaserScan lscan = toLaserScan(scan);
    pub.publish(lscan);
}

/**
 * Routes the packet to the correct decompressor
 */ 
void onRecieveRx(wireless_msgs::LoraPacket packet) {
    std::vector<uint8_t> data = uncompressZip(packet.data);
    if(data.size() < 0){
        ROS_ERROR("Failed to decompress packet");
        return;
    }
    switch(data[0]) {
        case (uint8_t)MessageType::LASER_SCAN:
            handleLaserScan(packet.from.data, data);
            break;
        default:
            ROS_ERROR("Handler not found");
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "basestation_relay");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    pub = nh.advertise<sensor_msgs::LaserScan>("/recieved/scan", 10);
    loraSub = nh.subscribe("/lora/rx", 10, &onRecieveRx);
    while(ros::ok()){
        ros::spinOnce();  
        rate.sleep();
    }
    return 0;
}