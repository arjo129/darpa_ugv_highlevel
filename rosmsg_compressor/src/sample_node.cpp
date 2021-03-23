#include <ros/ros.h>

#include <rosmsg_compressor/rosmsg_serializer.h>

// Includes for the various ROS message types that will be used / tested
#include <geometry_msgs/PointStamped.h>
#include <graph_msgs/GeometryGraph.h>

template <typename T>
class RosMsgSample
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

public:
    RosMsgSample()
    {
        pub = nh.advertise<T>("graph_pub", 10);
        sub = nh.subscribe("graph_listener", 10, &RosMsgSample::callback, this);
    }

    void callback(const T& msg)
    {
        ROS_INFO("Received msg..");

        // rosmsg -> bytes
        std::vector<uint8_t> buffer;
        RosMsgCompressor::serialize_to_byte_array(msg, buffer);

        // bytes -> rosmsg
        T deserialized_msg;
        RosMsgCompressor::deserialize_from_byte_array(buffer, deserialized_msg);

        
        ROS_INFO("Published msg..");
        pub.publish(deserialized_msg);
    }
};


int main (int argc, char **argv)
{
    ros::init(argc, argv, "rosmsg_compressor");
    
    /*
        You can use any ROS message as the template type for 
        `RosMsgSample` class as long as ROS message is included. 
    */
    //RosMsgSample<geometry_msgs::PointStamped> rsp;
    RosMsgSample<graph_msgs::GeometryGraph> rsp;

    ros::spin();
}