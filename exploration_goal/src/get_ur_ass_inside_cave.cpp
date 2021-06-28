#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <subt_msgs/PoseFromArtifact.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include "dmath/geometry.h"
#include <math.h>

#include "noroute_mesh/send_artifact.h"

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

bool first = true, goal_set = false, obstacle_avoidance_ready = false;
tf::Vector3 goal;
ros::ServiceClient client, artifact_client;
tf::TransformListener* listener;
ros::Publisher exploration_goal_pub, start_publisher;
std::string robot_name;
bool new_goal= true;
bool last_waypoint_completed = false;

std::vector <std::pair<float,float>> waypoints;
int waypoint_index = 0;


// void onFrontierAvailable(const sensor_msgs::PointCloud2::Ptr frontiers_ptr) {
//     if(!first || !obstacle_avoidance_ready) return;

//     ROS_INFO("Got frontier message");
//     pcl::PointCloud<pcl::PointXYZ> frontier;
//     pcl::fromROSMsg(*frontiers_ptr, frontier);    
//     tf::StampedTransform world_to_baselink;
//     try{
//         listener->waitForTransform(robot_name + "/base_link", robot_name+"/world", frontiers_ptr->header.stamp, ros::Duration(10));
//         listener->lookupTransform(robot_name + "/base_link", robot_name+"/world", frontiers_ptr->header.stamp, world_to_baselink);
//     } catch (tf::LookupException e){
//         ROS_ERROR("failed to lookup transform: %s", e.what());
//         return;
//     }

//     //auto status = ros::topic::waitForMessage<std_msgs::Int8>("status", node);

//     subt_msgs::PoseFromArtifact srv;
//     srv.request.robot_name.data = robot_name;

//     if(client.call(srv)) {
//         auto darpa_to_base_link = invertTransform(srv);
//         auto dir = darpa_to_base_link.getOrigin();
//         std::cout << "Got artifact_origin as" << dir.x() << "," << dir.y() << std::endl;
//         float max_dot = 0;

//         for(auto point: frontier) {
//             tf::Vector3 vec(point.x, point.y, point.z);
//             auto l = vec.dot(dir);
//             if(l > max_dot){
//                 max_dot = l;
//                 goal = vec;
//             }
//         }
//         std::cout << "Setting goal" <<goal.x() <<"," <<goal.y() <<std::endl;
//         first =false;
//         geometry_msgs::PointStamped stamp;
//         stamp.header.frame_id = robot_name + "world";
//         stamp.header.stamp = ros::Time::now();
//         stamp.point.x = 0.9*goal.x();
//         stamp.point.y = 0.9*goal.y();
//         stamp.point.z = 0.9*goal.z();
//         exploration_goal_pub.publish(stamp);
//         goal_set = true;
//     }
    
// }

void create_path(){
  waypoints.push_back (std::make_pair (10,-4));
  waypoints.push_back (std::make_pair (15,-4));
  waypoints.push_back (std::make_pair (15,-10));
  waypoints.push_back (std::make_pair (15,-20));
  waypoints.push_back (std::make_pair (15,-30));
  waypoints.push_back (std::make_pair (18,-40));
  waypoints.push_back (std::make_pair (18,-50));
  waypoints.push_back (std::make_pair (21,-52));
  std::cout <<  "Total waypoints: " << waypoints.size() << std::endl;
}

void pub_path(){

    // dmath::Vector3D goal_lc;
    // tf::StampedTransform world_to_baselink;
    // geometry_msgs::PointStamped goal_msg_lc;
    // geometry_msgs::PointStamped goal_msg_gl;
    // auto now =ros::Time::now();
    // auto most_recent = ros::Time(0);

    // goal_msg_gl.header.frame_id =  robot_name + "/world";
    // goal_msg_gl.point.x = waypoints[waypoint_index].first;
    // goal_msg_gl.point.y = waypoints[waypoint_index].second;
    // goal_msg_gl.point.z = 0;

    // if (new_goal){
    //   exploration_goal_pub.publish(goal_msg_gl);  
    //   std::cout << "Publishing " << waypoint_index << " th waypoint:" << waypoints[waypoint_index].first << " " << waypoints[waypoint_index].second << std:: endl;
    //   new_goal = false;
    // }


    // // try{
    // //     listener->waitForTransform(robot_name + "/base_link", robot_name + "/world", now, ros::Duration(10));
    // //     listener->transformPoint(robot_name + "/base_link", goal_msg_gl, goal_msg_lc);
    // // } catch (tf::LookupException e){
    // //     ROS_ERROR("failed to lookup transform: %s", e.what());
    // //     return;
    // // }

    // // goal_lc = -dmath::Vector3D(goal_msg_lc.point.x, goal_msg_lc.point.y, goal_msg_lc.point.z);

    // // // robot 1m within goal
    // // if(magnitude(goal_lc) < 1) {
    // //     std::cout << "Goal reached!" << std:: endl;
    // //     waypoint_index++;
    // // }

    // tf::StampedTransform world_to_baselink_check;
    // try{
    //     listener->waitForTransform(robot_name + "/base_link", robot_name+"/world", most_recent, ros::Duration(10));
    //     listener->lookupTransform(robot_name + "/base_link", robot_name+"/world", most_recent, world_to_baselink_check);
    // } catch (tf::LookupException e){
    //     ROS_ERROR("1. failed to lookup transform: %s", e.what());
    //     return;
    // }

    // auto goal = world_to_baselink_check.getOrigin();

    // float distance = sqrt(pow((goal.x() - waypoints[waypoint_index].first),2) + pow((goal.y() - waypoints[waypoint_index].second),2));

    // if (distance <= 2){
    //     std::cout << "Goal reached!" << std:: endl;
    //     waypoint_index++;
    //     new_goal = true;
    // }

}

void handover(std_msgs::Int8 reached) {
    obstacle_avoidance_ready = true;
    // if(!goal_set) {
    //     ROS_INFO("Haven't reached cave interior. Ignoring.");
    //     return;
    // }

    // tf::StampedTransform world_to_baselink;
    // try{
    //     auto now =ros::Time::now();
    //     listener->waitForTransform(robot_name + "/base_link", robot_name+"/world", ros::Time(0), ros::Duration(10));
    //     listener->lookupTransform(robot_name + "/base_link", robot_name+"/world", ros::Time(0), world_to_baselink);
    // } catch (tf::LookupException e){
    //     ROS_ERROR("2. failed to lookup transform: %s", e.what());
    //     return;
    // }

    // auto pos = world_to_baselink.getOrigin();
    // ROS_INFO("Cave goal %f,%f; Current pos %f, %f", goal.x(), goal.y(), pos.x(), pos.y());
    // if(pos.dot(pos) > 49) {
    //     ROS_INFO("Inside the cave. Handing over to explorer");
    //     std_msgs::Empty empty;
    //     start_publisher.publish(empty);
    // }
  
}

// void test(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
void test(const geometry_msgs::Twist& msg){

    //  std::cout << "Test!" << std:: endl;

    dmath::Vector3D goal_lc;
    tf::StampedTransform world_to_baselink;
    geometry_msgs::PointStamped goal_msg_lc;
    geometry_msgs::PointStamped goal_msg_gl;
    auto now =ros::Time::now();
    auto most_recent = ros::Time(0);

    if (waypoint_index >= waypoints.size()){
      if (!last_waypoint_completed){
        last_waypoint_completed = true;
        noroute_mesh::send_artifact srv;

        srv.request.type = "TYPE_HELMET";
        srv.request.x = 7.833333;
        srv.request.y = -51.640000;
        srv.request.z = 0.900000;
        if (artifact_client.call(srv))
        {
          ROS_INFO("[Artifact-Report-Spam] %s artifact reported successfully");
        }
        else
        {
          ROS_ERROR("Failed to send reports");
        }
        std::cout << "Waypoint completed. " << std::endl;
        return;
      }
      return;
    }

    goal_msg_gl.header.frame_id =  robot_name + "/world";
    goal_msg_gl.point.x = waypoints[waypoint_index].first;
    goal_msg_gl.point.y = waypoints[waypoint_index].second;
    goal_msg_gl.point.z = 0;

    // if (new_goal && !last_waypoint_completed){
    //   exploration_goal_pub.publish(goal_msg_gl);  
    //   std::cout << "Publishing " << waypoint_index << " th waypoint:" << waypoints[waypoint_index].first << " " << waypoints[waypoint_index].second << std:: endl;
    //   new_goal = false;
    // }

    tf::StampedTransform world_to_baselink_check;
    try{
        listener->waitForTransform(robot_name+"/world", robot_name + "/base_link", most_recent, ros::Duration(10));
        listener->lookupTransform(robot_name+"/world", robot_name + "/base_link", most_recent, world_to_baselink_check);
    } catch (tf::LookupException e){
        ROS_ERROR("1. failed to lookup transform: %s", e.what());
        return;
    }

    auto goal = world_to_baselink_check.getOrigin();

    float distance = sqrt(pow((goal.x() - waypoints[waypoint_index].first),2) + pow((goal.y() - waypoints[waypoint_index].second),2));

    std::cout << "current pose: " << goal.x() << " " << goal.y() << std::endl;
    std::cout << "distance to goal: " << distance << std::endl;

    // if (new_goal && !last_waypoint_completed){
    //   new_goal = false;
    //   exploration_goal_pub.publish(goal_msg_gl);  
    //   std::cout << "Publishing " << waypoint_index << " th waypoint:" << waypoints[waypoint_index].first << " " << waypoints[waypoint_index].second << std:: endl;
    // } 
  
    if (!last_waypoint_completed){
      new_goal = false;
      exploration_goal_pub.publish(goal_msg_gl);  
      std::cout << "Publishing " << waypoint_index << " th waypoint:" << waypoints[waypoint_index].first << " " << waypoints[waypoint_index].second << std:: endl;
    }
    
    if (distance < 2.5){
      std::cout << "Goal reached!" << std:: endl;
      waypoint_index++;
      new_goal = true;
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_ur_ass_inside");
    ros::NodeHandle node;
    int rate = 10;
    ros::param::get("~robot_name", robot_name);
    
    ROS_INFO("waiting for frontier message");
    // auto s = node.subscribe("frontiers/local", 1, onFrontierAvailable);
    auto m = node.subscribe("status", 1, handover);
    // ros::Subscriber test_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("points", 1, test);
    ros::Subscriber test_sub = node.subscribe("cmd_vel", 1, test);

    exploration_goal_pub = node.advertise<geometry_msgs::PointStamped>("goal_to_explore", 1);
    start_publisher = node.advertise<std_msgs::Empty>("start_exploration", 1);
    client = node.serviceClient<subt_msgs::PoseFromArtifact>("/subt/pose_from_artifact_origin");
    artifact_client = node.serviceClient<noroute_mesh::send_artifact>("send_artifact");
    listener = new tf::TransformListener();
    
    ros::Duration(10).sleep();
    create_path();
    // ros::Rate r(10);
    // while (ros::ok){
    //   pub_path();
    //   // ros::spinOnce();
    //   // r.sleep();
    // }
    ros::Rate r(rate);
    // while (ros::ok()){
    //   pub_path();
    //   r.sleep();
    // }

    ros::spin();

    return 0;

}