#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <data_compressor/devices/laserscan.h>
#include <wireless_msgs/LoraPacket.h>

class CompressedTelemetrySender {

    ros::Publisher loraPub;
    ros::Subscriber laserscan;
    data_compressor::LaserScanCompressor scanCompressor;
    data_compressor::PacketSendQueue sendQueue;
    std::vector<wireless_msgs::LoraPacket> latestPacket;
    tf::TransformListener* tfListener;

    void compressScan(sensor_msgs::LaserScan scan){
        data_compressor::DataPacket pkt = scanCompressor.compress(scan);
        std::cout << pkt.data.size()  << ":" << scan.ranges.size()<<std::endl;
        std::cout << scanCompressor.decompress(pkt).ranges.size() <<std::endl;
        tf::StampedTransform trans;
        /*try {
            this->tfListener->lookupTransform("odom", "base_link", scan.header.stamp, trans);
            pkt.estimated_pose.position.x = trans.getOrigin().x();
            pkt.estimated_pose.position.y = trans.getOrigin().y();
            pkt.estimated_pose.position.z = trans.getOrigin().z();
            pkt.estimated_pose.orientation.x = trans.getRotation().x();
            pkt.estimated_pose.orientation.y = trans.getRotation().y();
            pkt.estimated_pose.orientation.z = trans.getRotation().z();
            pkt.estimated_pose.orientation.w = trans.getRotation().w();
            this->latestPacket = sendQueue.send(bs,"base_station");
        } catch (tf::TransformException ex) {

        }*/
                    data_compressor::ByteStream bs = pkt.serialize();

        this->latestPacket = sendQueue.send(bs,"base_station");
    }

public:
    CompressedTelemetrySender(ros::NodeHandle& nh, std::string address): sendQueue(address) {
        this->laserscan = nh.subscribe("/scan", 10, &CompressedTelemetrySender::compressScan, this);   
        this->loraPub = nh.advertise<wireless_msgs::LoraPacket>("/tx", 10);
        this->tfListener = new tf::TransformListener();
    }

    void publish(){
        std::cout << "publishing" << this->latestPacket.size()<< std::endl;
        for(wireless_msgs::LoraPacket& pkt: this->latestPacket){
            this->loraPub.publish(pkt);
        }
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "compression_node");
    ros::NodeHandle nh;
    CompressedTelemetrySender telemetry(nh, "husky1");
    ros::Rate rate(0.5);
    while(ros::ok()){
        ros::spinOnce();
        telemetry.publish();
        rate.sleep();
    }
    return 0;
}