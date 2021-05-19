#include <ros/ros.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
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
    cv_bridge::CvImagePtr input_bridge;
    cv::Mat image;
    cv::Point3d **arr;

    public:
    graph_colouring_tool(): it_(nh_){
        std::string image_topic = nh_.resolveName("/X1/front/optical/depth");
        sub_ = it_.subscribeCamera(image_topic, 1, &graph_colouring_tool::image_cb, this);
        pub_ = it_.advertise("image_out", 1);
    }

    void image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){
        cv::Mat rawImage, rectifiedImage;
        cv_bridge::CvImagePtr input_bridge;
        // try {
        //     input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        //     image = input_bridge->image;
        // }
        // catch (cv_bridge::Exception& ex){
        //     ROS_ERROR("[draw_frames] Failed to convert image");
        //     return;
        // }
        cam_model.fromCameraInfo(info_msg);

        cv::Size s;
        s = cam_model.fullResolution();
        printf("%d, %d\n",s.width,s.height);
        //get_vectors();

        // cam_model.rectifyImage(rawImage, rectifiedImage);
        // input_bridge->image = rectifiedImage;
        // pub_.publish(input_bridge->toImageMsg());
        printf("cx: %d, cy: %d\n", cam_model.cx(),cam_model.cy());
        //printf("CAPTURED\n");
    }
    
    void get_vectors(){
        cv::Point3d temp;
        for (int i=0; i<HEIGHT; i++){
            for (int j=0; j<WIDTH;j++){
                cv::Point2d uv(i,j);
                temp = cam_model.projectPixelTo3dRay(cam_model.rectifyPoint(uv));
                printf("i:%d j:%d | x:%d y:%d z:%d\n", i, j, temp.x, temp.y, temp.z);
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