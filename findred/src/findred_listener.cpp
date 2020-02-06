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
    cv::Mat drawing(cv::Size(640, 480), CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<cv::Point2i> contour;
    // std::cout << msg.contours.size() << std::endl;
    for (int i = 0; i < msg.contours.size(); i+=2) {
        contour.push_back(cv::Point2i((int)msg.contours[i] * 3, (int)msg.contours[i+1] * 2));
    }
    cv::Scalar col((int)msg.color[0], (int)msg.color[1], (int)msg.color[2]);
    std::vector<std::vector<cv::Point2i>> conts;
    conts.push_back(contour);
    drawContours(drawing, conts, 0, col, cv::FILLED);
    cv::namedWindow("test");
    cv::imshow("test", drawing);
    cv::waitKey(3);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "red_viz");
    ros::NodeHandle n;
    ros::Subscriber redSub = n.subscribe("/redobject", 1000, cb);
    ros::spin();
    return 0;
}