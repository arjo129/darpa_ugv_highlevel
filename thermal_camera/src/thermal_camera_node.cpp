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
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub1 = it.advertise("thermal/image_raw1", 1);
    image_transport::Publisher pub2 = it.advertise("thermal/image_raw2", 1);
    ros::Rate rate(10); // Adjust frequency at which you read camera

    while(ros::ok()){
        rate.sleep();
	Mat img1(160, 120, CV_8UC3, Scalar(0, 0, 0));
	Mat img2(160, 120, CV_8UC3, Scalar(0, 0, 0));
	getThermalImage(img1, img2);
        cv_ptr1->image = img1;
	cv_ptr2->image = img2;
        pub1.publish(cv_ptr1->toImageMsg());
	pub2.publish(cv_ptr2->toImageMsg());
    }
}
