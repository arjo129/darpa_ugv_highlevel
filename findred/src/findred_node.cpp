#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <findred/RedObject.h>
#include "std_msgs/String.h"
#include <vector>
#include <string>

class FindRed {
    ros::NodeHandle nh;
    ros::Subscriber colSub, depthSub;
    ros::Publisher redPub;
    cv::Mat depthImg;

std::vector<std::vector<cv::Point>> getContours(cv::Mat img) {
     cv::Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    Canny(img, canny_output, 50, 150, 3);
    findContours( img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
    return contours;
}

void cleanContours(std::vector<std::vector<cv::Point>> &contours) {
    for (auto it = contours.begin(); it != contours.end();) {
        if (contourArea(*it) < 150 && arcLength(*it, false) < 50) {
            contours.erase(it);
        } else {
            int size = (*it).size();
            for (auto j = (*it).begin(); j != (*it).end();) {
                for (int x = 0; x < size/48; x++) { //take one point from every 1/75 of the contour
                    if (j != (*it).end()) {
                        (*it).erase(j);
                    }
                }
                if (j!= (*it).end()) {
                    j++;
                }
            }
            it++;
        }
    }
}

std::vector<cv::Point2f> getContourCenter(std::vector<std::vector<cv::Point>> &contours) {
    std::vector<cv::Moments> mu(contours.size());
    for( int i = 0; i<contours.size(); i++ )
    { mu[i] = moments( contours[i], false ); }
    std::vector<cv::Point2f> mc(contours.size());
    for( int i = 0; i<contours.size(); i++) { 
        mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
    }
    return mc;
}

void rs_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
    cv::Mat imgHSV, imgThresholded, imgThresholded1;
    cv::Mat mask (cv_ptr->image.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat drawing (cv_ptr->image.size(), CV_8UC3, cv::Scalar(0,0,0));
    cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
    cvtColor(cv_ptr->image, imgHSV, cv::COLOR_BGR2HSV);

//FOR CALIBRATION ------------------------------

//  int iLowH = 0;
//  int iHighH = 179;

//  int iLowS = 0; 
//  int iHighS = 255;

//  int iLowV = 0;
//  int iHighV = 255;
// int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
//     cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

// cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
//  cv::createTrackbar("HighH", "Control", &iHighH, 179);
//   cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
//  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

//  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
//  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

//--------------------------------------------------------
    cv::Scalar redHigh(80, 80, 255);
    cv::Scalar redLow(0, 0, 125);
    cv::Scalar redH(10, 255, 255);
    cv::Scalar redL(0, 50, 30);
    cv::inRange(imgHSV, cv::Scalar(0, 50, 30), cv::Scalar(10, 255, 255), imgThresholded);
    cv::inRange(imgHSV, cv::Scalar(170, 50, 30), cv::Scalar(179, 255, 255), imgThresholded1);
    cv::bitwise_or(imgThresholded, imgThresholded1, imgThresholded, cv::noArray());

    erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );


    std::vector<std::vector<cv::Point>> contours = getContours(imgThresholded);
    cleanContours(contours);
    std::vector<cv::Point2f> mc = getContourCenter(contours);
    for (int i = 0; i < contours.size(); i++) {
        drawContours(mask, contours, i , cv::Scalar(255, 255, 255), cv::FILLED); 
        cv::Scalar avg = mean(cv_ptr->image, mask);
        drawContours(drawing, contours, i, avg, cv::FILLED);
        uint8_t depth = depthImg.at<uint16_t>(mc[i]);
        cv::putText(drawing, std::to_string(depth), mc[i], CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0xffff), 1, 8, false);
        mask = cv::Scalar(0, 0, 0);
        findred::RedObject rmsg; 
        rmsg.depth = depth;
        rmsg.frame = msg->header.seq;
        rmsg.color.push_back(avg.val[0]);
        rmsg.color.push_back(avg.val[1]);
        rmsg.color.push_back(avg.val[2]);
        for (int j = 0; j < contours[i].size(); j++) {
            rmsg.contours.push_back(contours[i][j].x);
            rmsg.contours.push_back(contours[i][j].y);
        }
        redPub.publish(rmsg);

    }
    cv::namedWindow("thresh");
    cv::namedWindow("color_img");
    cv::imshow("color_img", cv_ptr->image);
    cv::imshow("thresh", drawing);
    cv::waitKey(3);
}

void depth_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
    depthImg = cv_ptr->image;
    // std::vector<cv::Point2f> mc = getContourCenter(gContours);
    // for (int i = 0; i < gContours.size(); i++) {
    //     drawContours(cv_ptr->image, gContours, i , cv::Scalar(0xffff), 3, 8, cv::noArray(), 0, cv::Point());
    //     cv::putText(cv_ptr->image, std::to_string(cv_ptr->image.at<uint16_t>(mc[i])), mc[i], CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0xffff), 1, 8, false);
    // }
    // cv::namedWindow("depth");
    // cv::imshow("depth", cv_ptr->image);
}

public:
FindRed(ros::NodeHandle _nh) {
    this->nh = _nh;
    colSub = this->nh.subscribe("/camera/color/image_raw", 1000, &FindRed::rs_callback, this);
    depthSub = this->nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1000, &FindRed::depth_callback, this);
    redPub = this->nh.advertise<findred::RedObject>("redobject", 1000);
}

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "finding_red");
    ros::NodeHandle n;
    FindRed fr(n);
    // ros::Subscriber rsSub = n.subscribe("/camera/color/image_raw", 1000, rs_callback);
    // ros::Subscriber depthSub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 1000, depth_callback);
    // ros::Publisher redPub = n.advertise<findred::RedObject>("redobject", 1000);
    ros::spin();
    return 0;
}