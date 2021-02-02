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

#define MAX_TIME  15

enum StategyState {
  NAIVE , EXPLORATION
};

enum VehicleState {
 MOVING, AWAITING_INSTRUCTION
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
tf::TransformListener* listener;
tf::StampedTransform transform;
ros::Publisher pub, status_pub;
ros::Publisher pub_vel;
pcl::PointCloud<pcl::PointXYZ> out;

VehicleState current_state = AWAITING_INSTRUCTION; 
StategyState current_strategy = EXPLORATION;
ros::Time last_goal;

geometry_msgs::Vector3 goal;
geometry_msgs::Vector3 current;
float yaw_to_goal = 0;
int direction = -1;




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


float distanceFromOrigin(float *p, bool plane){

  if(plane){
    return std::sqrt((p[0]*p[0]) + (p[1]*p[1]));
  }else{
    return std::sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]));
  }
}
float getFreeSpaceScoreLR(std::vector<std::tuple<float,float>> &distances){
    float score = 0;
    for(auto &a : distances){
      if(std::get<1>(a) < 3.0){
        if(std::get<0>(a) < 0){
          score -= std::pow((M_PI - std::fabs(std::get<0>(a))) , 2)*std::pow((3.0-std::get<1>(a)),3);
          
        }else{
          score += std::pow((M_PI - std::fabs(std::get<0>(a))) , 2)*std::pow((3.0-std::get<1>(a)),3);
          
        }
      }
    }
    //std::printf("%f\n" , score);
    return score;
}

float getFreeSpaceScoreLRAgg(std::vector<std::tuple<float,float>> &distances){
    float score = 0;
    for(auto &a : distances){
      if(std::get<1>(a) < 3.0){
        if(std::get<0>(a) < 0){
          score -= std::pow((M_PI - std::fabs(std::get<0>(a))) , 2)*std::pow((3.0-std::get<1>(a)),3);
          if(std::get<1>(a) < 0.5){
            score -= 60;
          }
        }else{
          score += std::pow((M_PI - std::fabs(std::get<0>(a))) , 2)*std::pow((3.0-std::get<1>(a)),3);
          if(std::get<1>(a) < 0.5){
            score += 60;
          }
        }
      }
    }
    //std::printf("%f\n" , score);
    return score;
}

float getFreeSpaceScoreFB(std::vector<std::tuple<float,float>> &distances){
    float score = 0;
    for(auto &a : distances){
      if(std::get<1>(a) < 3.0){
        if(std::fabs(std::get<0>(a)) < (M_PI/4.0)){
          score -= std::pow((3.0-std::get<1>(a)),3);
        }else if(std::fabs(std::get<0>(a)) > (3*M_PI/4.0)){
          score += std::pow((3.0-std::get<1>(a)),3);
        }
      }
    }
    //std::printf("%f\n" , score);
    return score;
}


float getFrontFreenessScore(std::vector<std::tuple<float,float>> &distances){
    

    float lower_bound_dist = 1;
    float upper_bound_dist = 2;
    
    float lower_bound_angle = -M_PI/3;
    float upper_bound_angle = M_PI/3;

    float total = 0;
    float score = 0;

    float totalCount = 0;
    
    for(auto &a: distances){
	  totalCount += 1;
        if(std::get<0>(a) < upper_bound_angle && std::get<0>(a) > lower_bound_angle){
            total += (upper_bound_dist - lower_bound_dist);
            if(std::get<1>(a) > upper_bound_dist ){
                score += (upper_bound_dist - lower_bound_dist);
            }else if(std::get<1>(a) > lower_bound_dist ){
                score += (std::get<1>(a) - lower_bound_dist);
            }
        }
    }
    return score/total;
}
float getBackFreenessScore(std::vector<std::tuple<float,float>> &distances){
    

    float lower_bound_dist = 1;
    float upper_bound_dist = 2;
    
    float lower_bound_angle = -4*M_PI/6;
    float upper_bound_angle = 4*M_PI/6;

    float total = 0;
    float score = 0;

    float totalCount = 0;
    
    for(auto &a: distances){
	  totalCount += 1;
        if(std::get<0>(a) > upper_bound_angle || std::get<0>(a) < lower_bound_angle){
            total += (upper_bound_dist - lower_bound_dist);
            if(std::get<1>(a) > upper_bound_dist ){
                score += (upper_bound_dist - lower_bound_dist);
            }else if(std::get<1>(a) > lower_bound_dist ){
                score += (std::get<1>(a) - lower_bound_dist);
            }
        }
    }
    return score/total;
}

float getConstraintScore(std::vector<std::tuple<float,float>> &distances){
    float score = 0;
    for(auto &a : distances){
      if(std::get<1>(a) < 2.0){

          score += std::pow((M_PI - std::fabs(std::get<0>(a))) , 2)*(2.0/std::get<1>(a));
        
      }
    }
    //std::printf("%f\n" , score);
    return score;
}

float getAngularVelocity(std::vector<std::tuple<float,float>> &distances){
    
   if(std::sqrt((goal.x - current.x)*(goal.x - current.x) + (goal.y - current.y)*(goal.y - current.y)) < 2){
        return 0;    
   }
   
  float lrscore = getFreeSpaceScoreLR(distances);
  float lrscoreA = getFreeSpaceScoreLRAgg(distances);
  float fbscore = getFreeSpaceScoreFB(distances);
  float score = getConstraintScore(distances);
  float frontSpace = getFrontFreenessScore(distances);
  float backSpace = getBackFreenessScore(distances);

   if(yaw_to_goal > M_PI/3){
      return yaw_to_goal/M_PI*direction;
   }else{
      return yaw_to_goal/M_PI*direction*2 - lrscoreA/10000.0; 
   }
   return 0;
}

int haltOrCancelGoal(VehicleState &current_state , ros::Time &last_goal , geometry_msgs::Vector3 &goal, geometry_msgs::Vector3 &current){
   float distance_to_goal = std::sqrt((goal.x - current.x)*(goal.x - current.x) + (goal.y - current.y)*(goal.y - current.y));
    // std::cout << distance_to_goal <<std::endl;
   
   if(current_state ==  VehicleState::AWAITING_INSTRUCTION){
      return 1;
   }
   
   if( distance_to_goal< 2){
      return 1;  // Goal Reached
   }

   auto time_since_last_goal = ros::Time::now() - last_goal;
   if(time_since_last_goal > ros::Duration(MAX_TIME,0)) {
     return 2;  //Taking Too long to reach
   }

   return 0;

}

float getForwardVelocity(std::vector<std::tuple<float,float>> &distances){

  float distance_to_goal = std::sqrt((goal.x - current.x)*(goal.x - current.x) + (goal.y - current.y)*(goal.y - current.y));
   
   

  float lrscore = getFreeSpaceScoreLR(distances);
  float fbscore = getFreeSpaceScoreFB(distances);
  float score = getConstraintScore(distances);
  float frontSpace = getFrontFreenessScore(distances);
  float backSpace = getBackFreenessScore(distances);

   if(yaw_to_goal > M_PI/3){
        //Uturn mode
      if(score > 15000){
        //too constrained find more space
        if(fbscore < -2000 || frontSpace  < (backSpace - 0.2)){
            return -0.3;
          }else{
            return 0.2;
          }
      }else{
        if( distance_to_goal< 2){
          return 0.1;
        }else{
          if(fbscore < -2000 || frontSpace  < (backSpace - 0.2)){
            return -0.3;
          }else{
            return 0.1;
          }
        }
      }
   }
      float yawFactor = (M_PI/2 - yaw_to_goal)/M_PI;
      float constraintFactor = (25000.0-score)/25000;
      float obstacleFactor = (30000 - std::fabs(lrscore))/30000.0;
      return (yawFactor + constraintFactor + obstacleFactor)/3.0*0.8; 
   

}