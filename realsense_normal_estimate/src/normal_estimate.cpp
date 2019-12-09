#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>




cv::Mat offsetImageWithPadding(cv::Mat& originalImage, int offsetX, int offsetY){
        cv::Mat padded = cv::Mat(originalImage.rows + 2 * abs(offsetY), originalImage.cols + 2 * abs(offsetX), CV_32FC1, cv::Scalar(0));
        originalImage.copyTo(padded(cv::Rect(abs(offsetX), abs(offsetY), originalImage.cols, originalImage.rows)));
        return cv::Mat(padded,cv::Rect(abs(offsetX) + offsetX, abs(offsetY) + offsetY, originalImage.cols, originalImage.rows));
}


class NormalEstimator {
public:
    NormalEstimator();
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    image_geometry::PinholeCameraModel camera_calibration_;
    void onImageRecieved(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
};

NormalEstimator::NormalEstimator(): it_(nh_){
    sub_ = it_.subscribeCamera("/camera/depth/image_rect_raw", 1, &NormalEstimator::onImageRecieved, this);

}

void NormalEstimator::onImageRecieved(const sensor_msgs::ImageConstPtr& image_msg, 
        const sensor_msgs::CameraInfoConstPtr& info_msg) {
    camera_calibration_.fromCameraInfo(info_msg);
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }
    
    cv::Mat smoothed_image;
    cv::blur(image, smoothed_image, cv::Size(9,9));
    
    cv::Mat x_position = cv::Mat::zeros(smoothed_image.size(), CV_32FC1);
    cv::Mat y_position = cv::Mat::zeros(smoothed_image.size(), CV_32FC1);
    cv::Mat z_position = cv::Mat::zeros(smoothed_image.size(), CV_32FC1);

    for(int x = 0; x < smoothed_image.rows; x++){
        for(int y = 0; y < smoothed_image.cols; y++){
            uint16_t depth = smoothed_image.at<uint16_t>(x,y);
            cv::Point3d pt = camera_calibration_.projectPixelTo3dRay(cv::Point(x,y));
            Eigen::Vector3d pos(pt.x, pt.y, pt.z);
            pos /= pos.norm();
            pos *= depth;

            x_position.at<float>(x,y) = pos.x();
            y_position.at<float>(x,y) = pos.y();
            z_position.at<float>(x,y) = pos.z();

        }
    }

    cv::Mat kernel = cv::Mat::ones(cv::Size(9,9), CV_32FC1);
    cv::Mat x_mean = cv::Mat::zeros(smoothed_image.size(), CV_32FC1);
    cv::Mat y_mean = cv::Mat::zeros(smoothed_image.size(), CV_32FC1);
    cv::Mat z_mean = cv::Mat::zeros(smoothed_image.size(), CV_32FC1);
    cv::filter2D(x_position, x_mean, -1, kernel);
    cv::filter2D(y_position, y_mean, -1, kernel);
    cv::filter2D(z_position, z_mean, -1, kernel);

    
}

int main(int argc,char** argv) {
    ros::init(argc, argv, "realsense_normal_estimates");
    NormalEstimator ne;
    ros::spin();
}