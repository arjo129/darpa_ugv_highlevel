#include <ros/ros.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
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
    
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformBroadcaster br;
    tf2_ros::TransformListener* ls;

    image_transport::ImageTransport it_;
    image_geometry::PinholeCameraModel cam_model;

    geometry_msgs::TransformStamped transformStamped;

    cv_bridge::CvImagePtr input_bridge;
    cv::Mat image;

    std::vector<cv::Point3d> arr;

    public:
    graph_colouring_tool(): it_(nh_){
        std::string image_topic = nh_.resolveName("/X1/front/optical/image_raw");
        sub_ = it_.subscribeCamera(image_topic, 1, &graph_colouring_tool::image_cb, this);
        pub_ = it_.advertise("image_out", 1);
        ls = new tf2_ros::TransformListener(tfBuffer_);
        get_vectors();
    }

    void image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){
        cam_model.fromCameraInfo(info_msg);
        //printf("%s\n", cam_model.tfFrame());
        tf2::Vector3 temp,test;
        temp.setX(1.0);
        temp.setY(1.0);
        temp.setZ(1.0);
        temp.setW(1.0);
        // transform all the saved points;
        try{
            transformStamped = tfBuffer_.lookupTransform(cam_model.tfFrame(), "world", ros::Time(0));
            test = tfBuffer_.transform(temp,"world");
            printf("x:%lf y:%lf z:%lf\n",test.x(), test.y(), test.z());
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could NOT transform camera to world frame: %s", ex.what());
        }
    }
    
    void get_vectors(){
        cv::Point3d temp;
        for (double i=0; i<HEIGHT; i++){
            for (double j=0; j<WIDTH;j++){
                cv::Point2d uv(i,j);
                temp = cam_model.projectPixelTo3dRay(cam_model.rectifyPoint(uv));
                printf("i:%lf j:%lf | x:%lf y:%lf z:%lf\n", i, j, temp.x, temp.y, temp.z);
                arr.push_back(temp);
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