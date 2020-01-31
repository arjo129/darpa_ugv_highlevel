#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>

std::vector<std::vector<cv::Point>> gContours;

std::vector<std::vector<cv::Point>> getContours(cv::Mat img) {
     cv::Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    Canny(img, canny_output, 50, 150, 3);
    findContours( img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
    return contours;
}

void cleanContours(std::vector<std::vector<cv::Point>> &contours) {
    for (auto it = contours.begin(); it != contours.end();) {
        if (contourArea(*it) < 150 && arcLength(*it, false) < 30) {
            contours.erase(it);
        } else {
            int size = (*it).size();
            for (auto j = (*it).begin(); j != (*it).end();) {
                for (int x = 0; x < size/75; x++) {
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


void rs_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
    cv::Mat imgHSV, imgThresholded, imgThresholded1;
    cv::Mat drawing (cv_ptr->image.size(), CV_8UC3, cv::Scalar(255,255,255));
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
    cv::inRange(imgHSV, redL, redH, imgThresholded);
    cv::inRange(imgHSV, cv::Scalar(170, 50, 30), cv::Scalar(179, 255, 255), imgThresholded1);
    cv::bitwise_or(imgThresholded, imgThresholded1, imgThresholded, cv::noArray());

    erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );


    std::vector<std::vector<cv::Point>> contours = getContours(imgThresholded);
    cleanContours(contours);
    gContours = contours;
    for (int i = 0; i < contours.size(); i++) {
        drawContours(drawing, contours, i , cv::Scalar(0, 0, 0), 3, 8, cv::noArray(), 0, cv::Point());
        std::cout << contours[i].size() << std::endl;
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
    for (int i = 0; i < gContours.size(); i++) {
        drawContours(cv_ptr->image, gContours, i , cv::Scalar(0, 0, 255), 3, 8, cv::noArray(), 0, cv::Point());
    }
    cv::namedWindow("depth");
    cv::imshow("depth", cv_ptr->image);
}

//Extremely misaligned. Need the aligned frames to work

int main(int argc, char **argv) {
    ros::init(argc, argv, "finding_red");
    ros::NodeHandle n;
    ros::Subscriber rsSub = n.subscribe("/camera/color/image_raw", 1000, rs_callback);
    ros::Subscriber depthSub = n.subscribe("/camera/depth/image_rect_raw", 1000, depth_callback);
    ros::spin();
    return 0;
}