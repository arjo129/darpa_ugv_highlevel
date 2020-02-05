#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <findred/RedObject.h>
#include <vector>
#include "std_msgs/String.h"
#include <string>

void cb(const findred::RedObject &msg) {
    ROS_INFO("I HEARD SOMETHING");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "red_viz");
    ros::NodeHandle n;
    // ros::Subscriber rsSub = n.subscribe("/camera/color/image_raw", 1000, rs_callback);
    ros::Subscriber depthSub = n.subscribe("/redobject", 1000, cb);
    ros::spin();
    return 0;
}