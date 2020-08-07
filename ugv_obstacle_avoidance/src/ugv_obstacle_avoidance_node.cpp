#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int8.h>

#define _lowerBound -15.0
#define _upperBound 15.0
#define nScanRings 16
#define MAX_TIME  30

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub, status_pub;
ros::Publisher pub_vel;
pcl::PointCloud<pcl::PointXYZ> out;

int global_i = 0;
geometry_msgs::Vector3 goal;
geometry_msgs::Vector3 current;

float yaw_to_goal = 0;
int direction = -1;

enum VehicleState {
 MOVING, AWAITING_INSTRUCTION
};

VehicleState current_state = AWAITING_INSTRUCTION; 
ros::Time last_goal;

int getRowVal(int angle_seg, int ring){
  return ring*40+angle_seg;
}

float getDistanceBetween(pcl::PointXYZ p1 , pcl::PointXYZ p2){
    return std::sqrt( (p2.z- p1.z) * (p2.z- p1.z) + (p2.y - p1.y) * (p2.y- p1.y) + (p2.x - p1.x) * (p2.x - p1.x) );
}

float getFreeSpaceScoreLR(std::vector<std::tuple<float,float>> &distances){
    float score = 0;
    for(auto &a : distances){
        if(std::get<1>(a) < 2.0){
          if(std::get<0>(a) < 0){
            score -= std::pow((M_PI - std::fabs(std::get<0>(a))) , 2)*(2.0/std::get<1>(a));
          }else{
            score += std::pow((M_PI - std::fabs(std::get<0>(a))) , 2)*(2.0/std::get<1>(a));
          }
        }
    }
    //std::printf("%f\n" , score);
    return score;
}

float getConstraintScore(std::vector<std::tuple<float,float>> &distances){

}

float getAngularVelocity(std::vector<std::tuple<float,float>> &distances){

  if(current_state == AWAITING_INSTRUCTION) return 0;
    
   if(std::sqrt((goal.x - current.x)*(goal.x - current.x) + (goal.y - current.y)*(goal.y - current.y)) < 1){
        return 0;    
   }
   
   float score = getFreeSpaceScoreLR(distances);
  std::cout << yaw_to_goal <<std::endl;
   return yaw_to_goal/M_PI*1.0*direction - score/10000.0;
}

float getForwardVelocity(std::vector<std::tuple<float,float>> &distances){

  if(current_state == VehicleState::AWAITING_INSTRUCTION) return 0;

    float distance_to_goal = std::sqrt((goal.x - current.x)*(goal.x - current.x) + (goal.y - current.y)*(goal.y - current.y));
    std::cout << distance_to_goal <<std::endl;
   if( distance_to_goal< 1){
        std_msgs::Int8 i;
        i.data = 0;
        status_pub.publish(i);
        current_state = VehicleState::AWAITING_INSTRUCTION; 
        return 0;    
   }
   if(yaw_to_goal > M_PI/2){
        return 0;
   }

   auto time_since_last_goal = ros::Time::now() - last_goal;
   if(time_since_last_goal > ros::Duration(MAX_TIME,0)) {
     std_msgs::Int8 i;
        i.data = -1;
        status_pub.publish(i);
        current_state = VehicleState::AWAITING_INSTRUCTION; 
     return 0;
   }
   
   float yawFactor = 2*(M_PI/2 - yaw_to_goal)/M_PI*(M_PI/2 - yaw_to_goal)/M_PI*0.8*0.5;
   float distanceFactor = 0.8;
   
   if(distance_to_goal < 2){
        distanceFactor = (distance_to_goal)/2*0.8;
   }
   distanceFactor *= 0.5;
   return distanceFactor + yawFactor;

}

float distanceFromOrigin(float *p, bool plane){

  if(plane){
    return std::sqrt((p[0]*p[0]) + (p[1]*p[1]));
  }else{
    return std::sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]));
  }
}

bool steep(float *p1 , float *p2){
  float angle = std::atan(  (p2[2]- p1[2]) / std::sqrt((p2[1]- p1[1]) * (p2[1]- p1[1]) + (p2[0]- p1[0]) * (p2[0]- p1[0]) )  );


  if(angle > 0.87 || distanceFromOrigin(p2 , true) < distanceFromOrigin(p1 ,true)  ||(p2[2]- p1[2]) < -0.3){
    return true;
  }else{
    return false;
  }
}

void getMaxTraversablePoint(float (*data)[10000][3] , int i , pcl::PointXYZ *point){
  int capture = 0;
  for(int ring = 0; ring < 14 ; ring++){
    if(steep(data[ring][i] , data[ring+1][i])){
      break;
    }
    capture++;
  }
      point->x = data[capture][i][0];
      point->y = data[capture][i][1];
      point->z = 0;
      //point->intensity = 10;
  
}

void positionCallBack(const nav_msgs::Odometry::ConstPtr& msg){
    nav_msgs::Odometry poseMsg = *msg;
    geometry_msgs::Quaternion currentPose;
    std::cout << "Got position fix" <<std::endl;
    current.x = poseMsg.pose.pose.position.x;
    current.y = poseMsg.pose.pose.position.y;
    current.z = poseMsg.pose.pose.position.z;
    
    currentPose.x = poseMsg.pose.pose.orientation.x;
    currentPose.y = poseMsg.pose.pose.orientation.y;
    currentPose.z = poseMsg.pose.pose.orientation.z;
    currentPose.w = poseMsg.pose.pose.orientation.w;
    
    std::cout << "Current goal" << goal.x << ", " << goal.y << "| " << current.x << ", " << current.y <<std::endl;
    
    tf2::Quaternion rotation(
    poseMsg.pose.pose.orientation.x,
    poseMsg.pose.pose.orientation.y,
    poseMsg.pose.pose.orientation.z,
    poseMsg.pose.pose.orientation.w);
    tf2::Vector3 vector(1, 0, 0);
    tf2::Vector3 rotated_vector = tf2::quatRotate(rotation, vector);
    
    tf2::Quaternion rotationRef(0,0,0.05, 0.999);
    tf2::Vector3 ref_vector = tf2::quatRotate(rotationRef, rotated_vector);
    
    tf2::Vector3 goalVec(
        goal.x - current.x,
        goal.y - current.y,
        0
    );
    
    rotated_vector.setZ(0);
    ref_vector.setZ(0);
    yaw_to_goal = tf2::tf2Angle(goalVec , rotated_vector);
    float refAngle1 = tf2::tf2Angle(goalVec , ref_vector);
    
    if(refAngle1 > yaw_to_goal){
        direction = -1;
    }else{
        direction  = 1;
    }
    
    
    
    //std::printf("%f\n" , yaw_to_goal);
    
        
 
}

void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
    current_state = VehicleState::MOVING;
    last_goal = ros::Time::now();
    geometry_msgs::PointStamped goalPosition = *msg;
    goal.x = goalPosition.point.x;
    goal.y = goalPosition.point.y;
    goal.z = 0;

    std::cout << "Got goal" << goal.x << ", " << goal.y <<std::endl;
    //std::printf("%f\t%f\t%f\n" , goal.x, goal.y , goal.z);
}

float getYaw( pcl::PointXYZ &point ){

    float flat_angle = std::atan(point.y/point.x);
    
    if(point.y > 0 && point.x > 0){
      flat_angle += M_PI;
    }else if(point.y > 0 && point.x < 0){
      flat_angle += 2 * M_PI;
    }else if(point.y < 0 && point.x < 0){
      
    }else if(point.y < 0 && point.x > 0){
      flat_angle += M_PI;
    }

    flat_angle -= (M_PI);

    //std::cout << flat_angle << ", "  <<atan2(point.y, point.x) <<std::endl;
    return flat_angle;

}

void callback(const PointCloud::ConstPtr& msg){
  geometry_msgs::Transform trans;
  trans.translation.x = 0;
  trans.translation.y = 0;
  trans.translation.z = 0;
  trans.rotation.x = 0;
  trans.rotation.y = 0;
  trans.rotation.z = 0;
  trans.rotation.w = 1;
  pcl_ros::transformPointCloud	(*msg , out , trans);
  std::vector<std::tuple<float,float>>distances;
   size_t cloudSize = out.size();
  int count = 0;
  // extract valid points from input cloud


  for (int i = 0; i < cloudSize; i++) {

    pcl::PointXYZ point;
    point.x = out[i].x;
    point.y = out[i].y;
    point.z = out[i].z;

    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    count ++;

    float flat_angle = getYaw(point);
    float dist = std::sqrt(point.x*point.x+point.y*point.y);

	distances.push_back(std::tuple<float,float>(flat_angle,dist));

  }
  
  std::sort(distances.begin() , distances.end());

 
  //TODO Remove the below comments
  float v = getForwardVelocity(distances);
  float w = getAngularVelocity(distances);
  
  //float score = getFreeSpaceScoreLR(distances);
  
  
  //printf("Score: %f\n" , score);
  geometry_msgs::Twist t;

  t.linear.x = v;
  t.angular.z = w;

  // pcl::PCLPointCloud2 pc2;
  // pcl::toPCLPointCloud2 (out ,pc2);
  // pub.publish(pc2);
  pub_vel.publish(t);

//  out.clear();
  // pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  // pcl::fromROSMsg(*msg, laserCloudIn);
  // pub.publish(laserCloudIn);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  std::printf("subscriebrs ");
  ros::Subscriber sub = nh.subscribe<PointCloud>("traversable_pointcloud_input", 1, callback);
  ros::Subscriber sub3 = nh.subscribe("robot_position_pose", 1, positionCallBack);
  ros::Subscriber sub2 = nh.subscribe("goal_to_explore", 1, goalCallBack);
  std::printf("subscriebrs ");
  pub_vel = nh.advertise<geometry_msgs::Twist> ("X1/cmd_vel", 1);
  status_pub = nh.advertise<std_msgs::Int8> ("/status", 1);
  ros::spin();
}


