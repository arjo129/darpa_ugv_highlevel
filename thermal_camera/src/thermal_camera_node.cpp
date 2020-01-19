#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

class ThermalManager {
    ros::NodeHandle nh;
    ros::Subscriber thermalSub, rsSub;
    bool thermalSignature = false;
    float thermalAng = 0; //in radians

float calcAngle (float x, float y) {
    float angle = atan(y/x);
    if (y < 0 && x < 0) {
        angle -= M_PI;
    }
    if (x < 0 && y > 0) {
        angle += M_PI;
    }
    return angle;
}

void image_callBack(const sensor_msgs::ImageConstPtr &msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
    cv::Mat raw_img = cv_ptr->image;
    cv::Mat thr;
    threshold(raw_img, thr, 25, 255, cv::THRESH_BINARY);
    cv::Moments m = moments(thr,true);
    cv::Point p(m.m10/m.m00, m.m01/m.m00);
    std::cout<< (int)p.x - 16 << " " << (int)p.y - 12<< std::endl;
    if (p.x > 0 && p.y > 0) {
        float x = p.x - 16;
        float y = p.y - 12;
        thermalSignature = true;
        thermalAng = calcAngle(x, y);
        std::cout << thermalAng * (180 / M_PI) << std::endl;
    }
    circle(raw_img, p, 1, cv::Scalar(255), -1);
    cv::resize(raw_img, raw_img, cv::Size(640, 480), 20, 20, 1);
    cv::namedWindow("test_window");
	cv::imshow("test_window", raw_img);
	cv::waitKey(3);	
}

void contours(cv::Mat img) {
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    Canny(img, canny_output, 50, 150, 3);
    findContours( img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
    //clean contours
    for (int i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) < 100 && arcLength(contours[i], false) < 30) {
            contours.erase(contours.begin() + i);
        }
    }
    // get the moments
    std::vector<cv::Moments> mu(contours.size());
    std::vector<cv::Moments> shortlisted(contours.size());
    for( int i = 0; i<contours.size(); i++ )
    { mu[i] = moments( contours[i], false ); }
    
    // get the centroid of figures.
    std::vector<cv::Point2f> mc(contours.size());
    for( int i = 0; i<contours.size(); i++) { 
        mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
    }
    
    
    // draw contours
    cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));
    for( int i = 0; i<contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(0,0,0);
        cv::Scalar colorC;
    drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
    float x = mc[i].x - 320;
    float y = mc[i].y - 240;
    if (abs(calcAngle(x, y) - thermalAng) < 0.15) {
        colorC = cv::Scalar(0, 0, 225);
    } else {
        colorC = cv::Scalar(255, 255, 255);
    }
    circle( drawing, mc[i], 4, colorC, -1, 8, 0 );
    }
    
    // show the resultant image
    cv::namedWindow( "Contours");
    cv::imshow( "Contours", drawing );
    cv::waitKey(3);

}

void rs_callBack(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
    //if (thermalSignature) {
        cv::Mat newimg(640, 480, CV_8UC1, cv::Scalar(0));
        cv_ptr->image.convertTo(newimg, CV_8UC1, 0.1);
        contours(newimg);
    //thermalSignature = false;
    }

public:
ThermalManager(ros::NodeHandle _nh) {
    this->nh = _nh;
    thermalSub = this->nh.subscribe("thermal_front/image_raw", 1000, &ThermalManager::image_callBack, this);
    rsSub = this->nh.subscribe("/camera/depth/image_rect_raw", 1000, &ThermalManager::rs_callBack, this);
}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
    ThermalManager tm(n);
	ros::spin();
	return 0;
}
