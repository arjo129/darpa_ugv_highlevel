#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>

class ThermalManager {
    ros::NodeHandle nh;
    ros::Subscriber thermalSub, rsSub;
    bool thermalSignature = false; //true if there is an unprocessed thermal frame with a hot object
    float thermalAng = 0; //in radians
    std::vector<float> thermalAngs;
    std::vector<std::vector<cv::Point>> thermalContours;
    bool center = false;

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

std::vector<float> getThermalAngs (std::vector<cv::Point2f> &thermalCenters) {
    std::vector<float> angles;
    for (int i = 0; i < thermalCenters.size(); i++) {
        angles.push_back(calcAngle(thermalCenters[i].x, thermalCenters[i].y));
    }
    return angles;
}

void image_callBack(const sensor_msgs::ImageConstPtr &msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
    cv::Mat raw_img = cv_ptr->image;
    cv::Mat thr;
    threshold(raw_img, thr, 35, 255, cv::THRESH_BINARY);
    thermalContours = getContours(thr);
    if (thermalContours.size() > 0) {
        thermalSignature = true;
        std::vector<cv::Point2f> thermalCenters = getContourCenter(thermalContours);
        thermalAngs = getThermalAngs(thermalCenters);
        for (int i = 0; i <  thermalCenters.size(); i++) {
            circle(raw_img, thermalCenters[i], 1, cv::Scalar(255), -1);
        }
    }
    cv::resize(raw_img, raw_img, cv::Size(640, 480), 20, 20, 1);
    cv::namedWindow("test_window");
	cv::imshow("test_window", raw_img);
	cv::waitKey(3);	
}

std::vector<std::vector<cv::Point>> getContours(cv::Mat img) {
     cv::Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    Canny(img, canny_output, 50, 150, 3);
    findContours( img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
    return contours;
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

void cleanContours(std::vector<std::vector<cv::Point>> &contours) {
    for (int i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) < 150 && arcLength(contours[i], false) < 30) {
            contours.erase(contours.begin() + i);
        }
    }
}

void shortlistPoints(std::vector<cv::Point2f> &centers) {
    std::cout << thermalAngs.size() << std::endl;
    for (auto it = centers.begin(); it != centers.end();) {
        for (int j = 0; j < thermalAngs.size(); j++) {
            // std::cout << (calcAngle((*it).x, (*it).y) - thermalAngs[j]) << std::endl;
            if (abs(calcAngle((*it).x, (*it).y) - thermalAngs[j]) > 0.08) {
                centers.erase(it);
            } else {
                ++it;
            }
        }
    }
}

void displayContours(std::vector<std::vector<cv::Point>> &contours, std::vector<cv::Point2f> &center, cv::Size img, std::string name) {
    cv::Mat drawing (img, CV_8UC3, cv::Scalar(255,255,255));
    for (int i = 0; i < contours.size(); i++) {
        drawContours(drawing, contours, i , cv::Scalar(0, 0, 0), 1, 8, cv::noArray(), 0, cv::Point());
        for (int j = 0; j < thermalAngs.size(); j++) {
            if (abs(calcAngle(center[i].x, center[i].y) - thermalAngs[j]) < 0.08) {
                circle( drawing, center[i], 4, cv::Scalar(0, 0, 255), -1, 8, 0 );
            }
        }
    }
    cv::namedWindow(name);
    cv::imshow(name, drawing);
    cv::waitKey(3);
}

void rs_callBack(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
    if (thermalSignature) {
        cv::Mat newimg(640, 480, CV_8UC1, cv::Scalar(0));
        cv_ptr->image.convertTo(newimg, CV_8UC1, 0.1);
        std::vector<std::vector<cv::Point>> contours = getContours(newimg);
        cleanContours(contours);
        std::vector<cv::Point2f> mc = getContourCenter(contours);
        // shortlistPoints(mc);
        displayContours(contours, mc, cv_ptr->image.size(), "realsense");
        thermalSignature = false;
    }
}

/**
 * To do:
 * Use matchShapes to match the contours
 * Check the center of the contour to ensure that it is in the same area
 * --matchShapes does not seem to work well, try with humanoid figures next time
 **/

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
