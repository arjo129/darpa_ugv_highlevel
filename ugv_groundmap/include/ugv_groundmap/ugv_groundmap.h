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

#define _lowerBound -15.0
#define _upperBound 15.0
#define nScanRings 16

#define NODE_DIST 5

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Time last_goal;
ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ> out;

ros::Publisher pub2;
ros::Publisher pub3;
pcl::PointCloud<pcl::PointXYZ> out2;

float getDistanceBetween(pcl::PointXYZ p1 , pcl::PointXYZ p2){
    return std::sqrt( (p2.z- p1.z) * (p2.z- p1.z) + (p2.y - p1.y) * (p2.y- p1.y) + (p2.x - p1.x) * (p2.x - p1.x) );
}

float getDistanceFromOrigin(pcl::PointXYZ p1 ){
    return std::sqrt( (p1.z) * (p1.z) + ( p1.y) * (p1.y) + (p1.x) * (p1.x) );
}


float distanceFromOrigin(float *p, bool plane){
  if(plane){
    return std::sqrt((p[0]*p[0]) + (p[1]*p[1]));
  }else{
    return std::sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]));
  }
}

bool steep(float *p1 , float *p2){
  float angle = std::atan(  std::fabs(p2[2]- p1[2]) / std::sqrt((p2[1]- p1[1]) * (p2[1]- p1[1]) + (p2[0]- p1[0]) * (p2[0]- p1[0]) )  );

  if(angle > 0.70 || distanceFromOrigin(p2 , true) < distanceFromOrigin(p1 ,true)){
    return true;
  }else{
    return false;
  }
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

    return flat_angle;

}

void getMaxTraversablePoint(float (*data)[10000][3] , int i , pcl::PointXYZ *point){
  int capture = 0;
  for(int ring = 0; ring < 9 ; ring++){
    if(steep(data[ring][i] , data[ring+1][i])){
      break;
    }
    capture++;
  }
  point->x = data[capture][i][0];
  point->y = data[capture][i][1];
  point->z = 0;
  
}

bool sortFunc(std::pair<float , pcl::PointXYZ> p1 , std::pair<float , pcl::PointXYZ> p2){
  return p1.first < p2.first;
}
bool sortFunc2(std::tuple<float , float> p1 , std::tuple<float , float> p2){
  return std::get<0>(p1) < std::get<0>(p2);
}
// pcl::PointXYZ transformPointToWorld(pcl::PointXYZ point , tf::TransformListener* listener){

//   geometry_msgs::PointStamped initial_pt; 
//   initial_pt.header.frame_id = "X1";
//   initial_pt.point.x = point.x;
//   initial_pt.point.y = point.y;
//   initial_pt.point.z = point.z;
  
//   geometry_msgs::PointStamped transformStamped;
//   pcl::PointXYZ transformedPoint;
//   listener->transformPoint("/world", initial_pt , transformStamped);	
//   transformedPoint.x = transformStamped.point.x;
//   transformedPoint.y = transformStamped.point.y;
//   transformedPoint.z = transformStamped.point.z;

//   return transformedPoint;

// }

// float getSectionFreenessScore(std::vector<std::tuple<float,float>> &distances , pcl::PointXYZ p){
    
//     pcl::PointXYZ v_p;
    
//     v_p.x = p.x;
//     v_p.y = p.y;
//     v_p.z = 0;

//     //float v_p_mag = getMagnitude(v_p);
    
//     tf2::Vector3 v_p_unit(v_p.x / NODE_DIST, v_p.y / NODE_DIST,0);

//     float lower_bound_dist = NODE_DIST -1;
//     float upper_bound_dist = NODE_DIST + 1;

//     tf2::Quaternion rotation1(0,0,0.706,0.709);
//     tf2::Quaternion rotation2(0,0,-0.706,0.709);

//     tf2::Vector3 left_vector = tf2::quatRotate(rotation1, v_p_unit);
    
//     tf2::Vector3 right_vector = tf2::quatRotate(rotation2, v_p_unit);
    
//     pcl::PointXYZ cw_corner;
//     pcl::PointXYZ ccw_corner;
    
//     ccw_corner.x = left_vector.x() + v_p.x;
//     ccw_corner.y = left_vector.y() + v_p.y;
//     ccw_corner.z = 0;

//     cw_corner.x = right_vector.x() + v_p.x;
//     cw_corner.y = right_vector.y() + v_p.y;
//     cw_corner.z = 0;
    
//     float lower_bound_angle = getYaw(ccw_corner);
//     float upper_bound_angle = getYaw(cw_corner);

//     //std::cout << "Distance" <<  lower_bound_dist << " " <<  upper_bound_dist << std::endl;
    
//     if(lower_bound_angle > upper_bound_angle){
//         float temp = lower_bound_angle;
//         lower_bound_angle = upper_bound_angle;
//         upper_bound_angle  = temp;
//     }
    
//     float total = 0;
//     float score = 0;

//     float totalCount = 0;
    
//     for(auto &a: distances){
// 	    totalCount += 1;
//         if(std::get<0>(a) < upper_bound_angle && std::get<0>(a) > lower_bound_angle){
//             total += (upper_bound_dist - lower_bound_dist);
//             if(std::get<1>(a) > upper_bound_dist ){
//                 score += (upper_bound_dist - lower_bound_dist);
//             }else if(std::get<1>(a) > lower_bound_dist ){
//                 score += (std::get<1>(a) - lower_bound_dist);
//             }
//         }
//     }
    
//     return score/total;
// }

// int getBestNextNode(std::vector<std::tuple<float,float>> &distances , pcl::PointXYZ &p ){
    
//     pcl::PointXYZ v_p;
    
//     v_p.x = 1;
//     v_p.y = 0;
//     // v_p.z = p.z - origin.z;


//     //float v_p_mag = getMagnitude(v_p);
    
//     tf2::Vector3 v_p_unit(v_p.x , v_p.y , 0);

//     tf2::Quaternion rot1;
//     tf2::Quaternion rot2;
    
//     float bestScore = 0;
//     pcl::PointXYZ bestPoint;
    
//     for(int i = 0; i <= 90 ;i += 10){
        
//         rot1.setRPY(0 ,0 , i/180.0*M_PI);
//         rot2.setRPY(0 ,0 , -i/180.0*M_PI);
        
//         tf2::Vector3 CCW_vec = tf2::quatRotate(rot1, v_p_unit);
//         tf2::Vector3 CW_vec = tf2::quatRotate(rot2, v_p_unit);
        
//         pcl::PointXYZ v_intermediate_CCW( NODE_DIST * CCW_vec.x() ,  NODE_DIST * CCW_vec.y() , 0);
//         pcl::PointXYZ v_intermediate_CW( NODE_DIST * CW_vec.x() ,  NODE_DIST * CW_vec.y() , 0);
        
        
//         //std::cout << "Score " << score << " " << total <<std::endl;
//         pcl::PointXYZ zeroPoint;
        
//         float score1 = getSectionFreenessScore(distances , v_intermediate_CCW);
//         float score2 = getSectionFreenessScore(distances , v_intermediate_CW);
//         // std::cout << i <<  score1<< score2 << std::endl;
//         //std::cout << CW_vec.x() <<  " " << CW_vec.y() << " " << score1 << std::endl; 
//         //std::cout << CCW_vec.x() <<  " " << CCW_vec.y() << " " << score2 << std::endl; 
//         if(score1 > 0.95){
//             bestPoint =  v_intermediate_CCW;
//             p.x = bestPoint.x;
//             p.y = bestPoint.y;
//             bestScore = score1;
//             break;
//         }
//         if(score2 > 0.95){
//             bestPoint =  v_intermediate_CW;
//             bestScore = score2;
//             p.x = bestPoint.x;
//             p.y = bestPoint.y;

//             break;
//         }
        
//         if(score1 > bestScore){
//             bestPoint = v_intermediate_CCW;
//             bestScore = score1;
            
//         }
//         if(score2 > bestScore){
//             bestPoint = v_intermediate_CW;
//             bestScore = score2;
//         }
    
//     }
//     // std::cout <<"BEst Point" << bestPoint.x << " " << bestPoint.y << " " << bestPoint.z << " " << bestScore << std::endl;
//     // if(bestScore < 0.4){
//     //   return pcl::PointXYZ(0,0,0);
//     // }

//     if(bestPoint.x == 0 && bestPoint.y == 0 && bestPoint.z == 0){
//         p.x = -3;
//         p.y = 0;
//     }
    
// }

void transformPointToWorldAndPublish(pcl::PointXYZ point){

  tf::TransformListener* listener = new tf::TransformListener();
   tf::StampedTransform transform;
  try{
    listener->waitForTransform("X1/world", "X1", ros::Time(0), ros::Duration(3.0));
    listener->lookupTransform("X1/world", "X1", ros::Time(0), transform);


      geometry_msgs::PointStamped initial_pt; 
    initial_pt.header.frame_id = "X1";
    initial_pt.point.x = point.x;
    initial_pt.point.y = point.y;
    initial_pt.point.z = point.z;
    
    geometry_msgs::PointStamped transformStamped;
    pcl::PointXYZ transformedPoint;
    listener->transformPoint("X1/world", initial_pt , transformStamped);	

    pub3.publish(transformStamped);
  }
   catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", "X1/world", "X1", ex.what());
    
  }

  
  

}