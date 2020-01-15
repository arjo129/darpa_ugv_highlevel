#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <wireless_msgs/LoraPacket.h>

class BaseStationRelay {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "basestation_relay");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
      
        rate.sleep();
    }
    return 0;
}