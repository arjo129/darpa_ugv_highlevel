#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

/**
* Example feel free to change
*/
cv::Mat getThermalImage() {
    return cv::Mat::zeros(cv::Size(20,20), CV_32FC1);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_camera_driver");
    ros::NodeHandle nh;
    cv_bridge::CvImagePtr cv_ptr;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("thermal/image_raw", 1);
    ros::Rate rate(10); // Adjust frequency at which you read camera

    while(ros::ok()){
        rate.sleep();
        cv_ptr->image = getThermalImage();
        pub.publish(cv_ptr->toImageMsg());
    }
}