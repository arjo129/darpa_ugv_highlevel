/**
 * This implements the telemetry sent by the UGV 
 */ 
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <wireless_msgs/LoraPacket.h>
#include <data_compressor/msgs/laserscan.h>
#include <data_compressor/msgs/co2.h>
#include <data_compressor/msgs/WifiArray.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>


class CompressedTelemetrySender {

    ros::Publisher loraPub, estopPub, startPub;
    ros::Subscriber laserscan, co2, wifi, loraSubscriber, odomSub;
    nav_msgs::Odometry lastOdom;
    bool recieved_odom = false;
    tf::TransformListener* tfListener;
    wireless_msgs::LoraPacket scanPacket; 
    
    void compressScan(sensor_msgs::LaserScan scan) {
        if(!recieved_odom){
            return;
        }
        std::vector<AdaptiveTelemetryScan> result = convertLaserScan(scan, lastOdom, 4);
        scanPacket = toLoraPacket(result[0]);
        loraPub.publish(scanPacket);
    }

    void onRecieveOdom(nav_msgs::Odometry odom) {
        this->lastOdom = odom;
        this->recieved_odom = true;
    }

    void onRecieveLora(wireless_msgs::LoraPacket packet) {
        std::vector<uint8_t> data = uncompressZip(packet.data);
        if(data.size() < 0){
            ROS_ERROR("Failed to decompress packet");
            return;
        }
        if(data[0] == (uint8_t) MessageType::ESTOP){
            std_msgs::String str;
            str.data = "LoRA E-Stop";
            estopPub.publish(str);
            std::vector<uint8_t> response;
            wireless_msgs::LoraPacket respPacket;
            respPacket.to = packet.from;
            response.push_back((uint8_t)MessageType::ESTOP_ACK);
            respPacket.data = compressZip(response);
            loraPub.publish(respPacket);
            ROS_INFO("Stopping due to E-Stop signal");
            return;
        }
    }

public:
    CompressedTelemetrySender(ros::NodeHandle& nh) {
        //Laser scan compressor
        
        this->loraPub = nh.advertise<wireless_msgs::LoraPacket>("/lora/tx", 10);
        this->estopPub = nh.advertise<std_msgs::String>("/e_stop", 10);
        this->tfListener = new tf::TransformListener();
        this->laserscan = nh.subscribe("/scan", 10, &CompressedTelemetrySender::compressScan, this);
        this->odomSub = nh.subscribe("/odom", 10, &CompressedTelemetrySender::onRecieveOdom, this);
        //this->co2 = nh.subscribe("/co2", 10, &CompressedTelemetrySender::compressScan, this);
        //this->wifi = nh.subscribe("/wifi", 10, &CompressedTelemetrySender::compressScan, this);
        this->loraSubscriber = nh.subscribe("/lora/rx", 10, &CompressedTelemetrySender::onRecieveLora, this);  
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "compression_node");
    ros::NodeHandle nh;
    CompressedTelemetrySender telemetry(nh);
    ros::Rate rate(1.5);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
