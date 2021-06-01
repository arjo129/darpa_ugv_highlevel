#include <ros/ros.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include "tf2/buffer.h"
#include <tf2/transform_listener.h>
#include <tf2/transform_broadcaster.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#define HEIGHT  240
#define WIDTH   320

class graph_colouring_tool{
    ros::NodeHandle nh_;
    image_transport::CameraSubscriber sub_;
    image_transport::Publisher pub_;
    // ros::Subscriber sub_;

    image_transport::ImageTransport it_;
    image_geometry::PinholeCameraModel cam_model;

    tf2::Buffer tfBuffer;
    tf2::TransformListener tfListener(tfBuffer);
    tf2::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped;

    cv_bridge::CvImagePtr input_bridge;
    cv::Mat image;
    cv::Point3d **arr;

    public:
    graph_colouring_tool(): it_(nh_){
        std::string image_topic = nh_.resolveName("/X1/front/optical/image_raw");
        sub_ = it_.subscribeCamera(image_topic, 1, &graph_colouring_tool::image_cb, this);
        pub_ = it_.advertise("image_out", 1);
        get_vectors();
    }

    void image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){
        cam_model.fromCameraInfo(info_msg);
        //printf("%s\n", cam_model.tfFrame());
        
        try{
            transformedStamped = tfBuffer.lookupTransform(cam_model.tfFrame(),"world",ros::Time(0));
        }
        catch(tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

    }
    
    void get_vectors(){
        cv::Point3d temp;
        for (double i=0; i<HEIGHT; i++){
            for (double j=0; j<WIDTH;j++){
                cv::Point2d uv(i,j);
                temp = cam_model.projectPixelTo3dRay(cam_model.rectifyPoint(uv));
                if (i== 0.0 && j == 0.0)
                printf("i:%lf j:%lf | x:%lf y:%lf z:%lf\n", i, j, temp.x, temp.y, temp.z);
            }
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_colouring");
    graph_colouring_tool* gc;
    gc = new graph_colouring_tool();
    ros::spin();
}