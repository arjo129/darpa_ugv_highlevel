/***
 * Decodes packets coming in via LoRA and sends heartbeat status signal to all robots
 * 
 */  
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <wireless_msgs/LoraPacket.h> 
#include <wifi_sensor/wifiArray.h>
#include <data_compressor/zip.h>
#include <data_compressor/protocol.h>
#include <data_compressor/msgs/laserscan.h>
#include <data_compressor/msgs/co2.h>
#include <data_compressor/msgs/WifiArray.h>

ros::Publisher laserPub;
ros::Publisher co2Pub;
ros::Publisher waPub;
ros::Subscriber loraSub;
ros::NodeHandle *nh;

void handleLaserScan(std::string from, std::vector<uint8_t> data) {
    AdaptiveTelemetryScan scan = decodeScan(data);
    sensor_msgs::LaserScan lscan = toLaserScan(scan);
    laserPub.publish(lscan);
}

void handleCo2(std::string from, std::vector<uint8_t> data) {
    Co2 co2 = decodeCo2(data);
    wireless_msgs::Co2 msg = toCo2(co2);
    co2Pub.publish(msg);
}

void handleWifiArray(std::string from, std::vector<uint8_t> data) {
    WifiArray wa = decodeWifiArray(data);
    wireless_msgs::WifiArray msg = toWifiArray(wa);
    waPub.publish(msg);
}

void handleEStopAck(std::string from, std::vector<uint8_t> data) {

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
        case (uint8_t)MessageType::ESTOP_ACK:
            handleEStopAck(packet.from.data, data);
        case (uint8_t)MessageType::CO2_SIGNATURE:
            handleCo2(packet.from.data, data);
            break;
        case (uint8_t)MessageType::WIFI_SIGNAL:
            handleWifiArray(packet.from.data, data);
            break;
        default:
            ROS_ERROR("Handler not found");
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "basestation_relay");
    nh = new ros::NodeHandle();
    ros::Rate rate(10);
    laserPub = nh->advertise<sensor_msgs::LaserScan>("/recieved/scan", 10);
    co2Pub = nh->advertise<wireless_msgs::Co2>("/recieved/co2", 10);
    waPub = nh->advertise<wireless_msgs::Co2>("/recieved/wifi", 10);
    loraSub = nh->subscribe("/lora/rx", 10, &onRecieveRx);
    while(ros::ok()){
        ros::spinOnce();  
        rate.sleep();
    }
    delete nh;
    return 0;
}
