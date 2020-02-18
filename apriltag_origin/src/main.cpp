#include "stdlib.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

class apriltag_origin {
public:
    ros::Subscriber tag_sub;
    ros::Publisher pub;
    tf::TransformBroadcaster br;
    tf::TransformListener listen;

    tf::Transform transform;
    tf::Transform tr2;
    tf::StampedTransform d;
    tf::StampedTransform st2;
    std::string source_frame = "middle";
    std::string dest_frame = "base_link";
    
    int timeout = 100;
    float APRILTAG_TO_FLOOR_HEIGHT = 0.85;
    bool terminate_apriltag_detector = false;

    double _x, _y, _z;

    apriltag_origin(ros::NodeHandle nh)
    {
        ROS_INFO("Starting Apriltag Scan");
        tag_sub = nh.subscribe("/tag_detections", 1, &apriltag_origin::Callback, this);
        pub = nh.advertise<std_msgs::Bool>("DARPA_Frame_Up", 100);
        while (ros::ok()){
            ros::spinOnce();

            // once DARPA frame is published 100 times using callback, the loop below fixes the DARPA FRAME to location of map
            if (timeout == 0){
                if (!terminate_apriltag_detector){
                    terminate_apriltag_detector = true;
                    system("rosnode kill /apriltag_ros_continuous_node");
                    std::cout << "Killed apriltag_continuous_node";
                }
                br.sendTransform(tf::StampedTransform (tr2,ros::Time::now(),"darpa","map"));
                std::cout << "DARPA frame transform sent [PERMANENT]\n";
                std_msgs::Bool b;
                b.data = true;

                // Publish the message to say DARPA Frame is up, other processes can now take place
                pub.publish(b);
                ros::Duration(0.5).sleep();
            }
        }
    }


    void Callback(const apriltag_ros::AprilTagDetectionArrayPtr& msg)
    {
        if (timeout > 0){
            std::cout << "Size of detections: " << msg->detections.size() << "\n";
            if (msg->detections.size() >= 2) {
                std::vector<apriltag_ros::AprilTagDetection> temp;
                for (auto a : msg->detections) {
                    temp.push_back(a);
                }
                _x = (temp[0].pose.pose.pose.position.x  + temp[1].pose.pose.pose.position.x) / 2;
                _y = (temp[0].pose.pose.pose.position.y  + temp[1].pose.pose.pose.position.y) / 2;
                _z = (temp[0].pose.pose.pose.position.z  + temp[1].pose.pose.pose.position.z) / 2;

                std::cout << "x: " << _x << "\n";
                std::cout << "y: " << _y << "\n";
                std::cout << "z: " << _z << "\n";
                
                transform.setOrigin(tf::Vector3(_x,_y,_z));
                transform.setRotation(tf::Quaternion(0,0,0,1));

                tf::StampedTransform st(transform, ros::Time::now(),"camera_color_optical_frame","middle");
                br.sendTransform(st);

                try{
                    // Calculate the distance between (the middle of the tags) and (base_link)
                    // This assumption is made as map spawns at base_link
                    listen.lookupTransform(dest_frame,source_frame,ros::Time(0),st2);

                    // Correction done to have the DARPA frame be aligned to middle
                    // APRILTAG_TO_FLOOR_HEIGHT is added as documentation shows the height that the April Tags are elevated is 0.85m
                    tf::Vector3 v;
                    v = st2.getOrigin();
                    v.setX(v.getX());
                    v.setZ((v.getZ()) + APRILTAG_TO_FLOOR_HEIGHT);
                    v.setY(v.getY());
                    //v.setX(v.getX() * -1);
                    //v.setZ((v.getZ() * -1) + APRILTAG_TO_FLOOR_HEIGHT);
                    //v.setY(v.getY() * -1);
                    tr2.setOrigin(v);
                    tr2.setRotation(tf::Quaternion(0,0,0,1));

                    // use the final calculated vector to create DARPA Frame with map as the child Frame
                    br.sendTransform(tf::StampedTransform (tr2,ros::Time::now(),"darpa","map"));
                    std::cout << "DARPA frame transform sent\n";
                    timeout = timeout - 1;
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DARPA_Frame_Generation");
    ros::NodeHandle nh;
    apriltag_origin DarpaFrame(nh);
    return 0;
}