#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <subt_msgs/PoseFromArtifact.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>

tf::Transform invertTransform(subt_msgs::PoseFromArtifact & service)
{
  tf::Vector3 translation(
    service.response.pose.pose.position.x,
    service.response.pose.pose.position.y,
    service.response.pose.pose.position.z
    );
  tf::Quaternion rotation(
    service.response.pose.pose.orientation.x,
    service.response.pose.pose.orientation.y,
    service.response.pose.pose.orientation.z,
    service.response.pose.pose.orientation.w
    );

  tf::Transform originalTF((rotation), translation);
  auto invertedTF = originalTF.inverse();
  return invertedTF;
}

bool first = true;
ros::ServiceClient client;
tf::TransformListener* listener;
ros::Publisher exploration_goal_pub;

void onFrontierAvailable(const sensor_msgs::PointCloud2::Ptr frontiers_ptr) {
    if(!first) return;

    ROS_INFO("Got frontier message");
    pcl::PointCloud<pcl::PointXYZ> frontier;
    pcl::fromROSMsg(*frontiers_ptr, frontier);    
    tf::StampedTransform world_to_baselink;
    try{
        listener->waitForTransform("X1/base_link", "world", frontiers_ptr->header.stamp, ros::Duration(10));
        listener->lookupTransform("X1/base_link", "world", frontiers_ptr->header.stamp, world_to_baselink);
    } catch (tf::LookupException e){
        ROS_ERROR("failed to lookup transform: %s", e.what());
        return;
    }

    //auto status = ros::topic::waitForMessage<std_msgs::Int8>("status", node);

    subt_msgs::PoseFromArtifact srv;
    srv.request.robot_name.data = "X1";

    if(client.call(srv)) {
        auto darpa_to_base_link = invertTransform(srv);
        auto dir = darpa_to_base_link.getOrigin();
        std::cout << "Got artifact_origin as" << dir.x() << "," << dir.y() << std::endl;
        float max_dot = 0;
        tf::Vector3 goal;
        for(auto point: frontier) {
            tf::Vector3 vec(point.x, point.y, point.z);
            auto l = vec.dot(dir);
            if(l > max_dot){
                max_dot = l;
                goal = vec;
            }
        }
        std::cout << "Setting goal" <<goal.x() <<"," <<goal.y() <<std::endl;
        first =false;
        geometry_msgs::PointStamped stamp;
        stamp.header.frame_id = "world";
        stamp.header.stamp = ros::Time::now();
        stamp.point.x = 0.9*goal.x();
        stamp.point.y = 0.9*goal.y();
        stamp.point.z = 0.9*goal.z();
        exploration_goal_pub.publish(stamp);
    }
    
}

void handover(std_msgs::Int8 reached) {
    ROS_INFO("Inside the cave. Handing over to explorer");
    std_msgs::Empty empty;
    exploration_goal_pub.publish(empty);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_ur_ass_inside");
    ros::NodeHandle node;
    
    ROS_INFO("waiting for frontier message");
    auto s = node.subscribe("/frontiers/local", 1, onFrontierAvailable);
    auto m = node.subscribe("/status", 2, handover);
    exploration_goal_pub = node.advertise<geometry_msgs::PointStamped>("goal_to_explore", 1);
    auto start_publisher = node.advertise<std_msgs::Empty>("/start_exploration", 1);
    client = node.serviceClient<subt_msgs::PoseFromArtifact>("/subt/pose_from_artifact_origin");
    listener = new tf::TransformListener();
    ros::spin();
    
}