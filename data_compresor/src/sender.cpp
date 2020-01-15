#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <wireless_msgs/LoraPacket.h>
#include <data_compressor/msgs/laserscan.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>


class CompressedTelemetrySender {

    ros::Publisher loraPub;
    ros::Subscriber laserscan;
    tf::TransformListener* tfListener;
    wireless_msgs::LoraPacket scanPacket;


    void compressScan(sensor_msgs::LaserScan scan){
        nav_msgs::Odometry odom;
        std::vector<AdaptiveTelemetryScan> result = convertLaserScan(scan, odom, 4);
        scanPacket = toLoraPacket(result[0]);
    }

public:
    CompressedTelemetrySender(ros::NodeHandle& nh) {
        this->laserscan = nh.subscribe("/scan", 10, &CompressedTelemetrySender::compressScan, this);   
        this->loraPub = nh.advertise<wireless_msgs::LoraPacket>("/tx", 10);
        this->tfListener = new tf::TransformListener();
    }

    void publish(){
        static uint8_t count ;
        loraPub.publish(scanPacket);    
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "compression_node");
    ros::NodeHandle nh;
    CompressedTelemetrySender telemetry(nh);
    ros::Rate rate(1.5);
    while(ros::ok()){
        ros::spinOnce();
        telemetry.publish();
        rate.sleep();
    }
    return 0;
}