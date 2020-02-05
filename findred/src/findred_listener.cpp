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
    std::vector<cv::Point> contour;
    // std::cout << msg.contours.size() << std::endl;
    for (int i = 0; i < msg.contours.size(); i++) {
        cv::Point p;
        p.x = (int)msg.contours[i];
        p.y = (int)msg.contours[i+1];
        contour.push_back(p);
    }
    std::vector<std::vector<cv::Point>> conts;
    conts.push_back(contour);
    drawContours(drawing, conts, 0, cv::Scalar(255, 255 ,255), cv::FILLED);
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