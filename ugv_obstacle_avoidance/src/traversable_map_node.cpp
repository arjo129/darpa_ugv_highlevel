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
    // std::cout << p1.x << " " << p1.y << " " << p1.z <<std::endl;
    // std::cout << p2.x << " " << p2.y << " " << p2.z <<std::endl;
    return std::sqrt( (p2.z- p1.z) * (p2.z- p1.z) + (p2.y - p1.y) * (p2.y- p1.y) + (p2.x - p1.x) * (p2.x - p1.x) );
}

float getDistanceFromOrigin(pcl::PointXYZ p1 ){
    // std::cout << p1.x << " " << p1.y << " " << p1.z <<std::endl;
    // std::cout << p2.x << " " << p2.y << " " << p2.z <<std::endl;
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
      //point->intensity = 10;
  
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

float getSectionFreenessScore(std::vector<std::tuple<float,float>> &distances , pcl::PointXYZ p){
    
    pcl::PointXYZ v_p;
    
    v_p.x = p.x;
    v_p.y = p.y;
    v_p.z = 0;

    //float v_p_mag = getMagnitude(v_p);
    
    tf2::Vector3 v_p_unit(v_p.x / NODE_DIST, v_p.y / NODE_DIST,0);



    
    float lower_bound_dist = NODE_DIST -1;
    float upper_bound_dist = NODE_DIST + 1;

    tf2::Quaternion rotation1(0,0,0.706,0.709);
    tf2::Quaternion rotation2(0,0,-0.706,0.709);

    tf2::Vector3 left_vector = tf2::quatRotate(rotation1, v_p_unit);
    
    tf2::Vector3 right_vector = tf2::quatRotate(rotation2, v_p_unit);
    
    pcl::PointXYZ cw_corner;
    pcl::PointXYZ ccw_corner;
    
    ccw_corner.x = left_vector.x() + v_p.x;
    ccw_corner.y = left_vector.y() + v_p.y;
    ccw_corner.z = 0;

    cw_corner.x = right_vector.x() + v_p.x;
    cw_corner.y = right_vector.y() + v_p.y;
    cw_corner.z = 0;
    
    float lower_bound_angle = getYaw(ccw_corner);
    float upper_bound_angle = getYaw(cw_corner);

    //std::cout << "Distance" <<  lower_bound_dist << " " <<  upper_bound_dist << std::endl;
    
    if(lower_bound_angle > upper_bound_angle){
        float temp = lower_bound_angle;
        lower_bound_angle = upper_bound_angle;
        upper_bound_angle  = temp;
    }
    
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
    //std::cout << totalCount << std::endl;
    // std::cout << "Score " << score << " " << total <<std::endl;
    // std::cout << score/total << std::endl; 
    
    return score/total;
}

int getBestNextNode(std::vector<std::tuple<float,float>> &distances , pcl::PointXYZ &p ){
    
    pcl::PointXYZ v_p;
    
    v_p.x = 1;
    v_p.y = 0;
    // v_p.z = p.z - origin.z;


    //float v_p_mag = getMagnitude(v_p);
    
    tf2::Vector3 v_p_unit(v_p.x , v_p.y , 0);

    tf2::Quaternion rot1;
    tf2::Quaternion rot2;
    
    float bestScore = 0;
    pcl::PointXYZ bestPoint;
    
    for(int i = 0; i <= 90 ;i += 10){
        
        rot1.setRPY(0 ,0 , i/180.0*M_PI);
        rot2.setRPY(0 ,0 , -i/180.0*M_PI);
        
        tf2::Vector3 CCW_vec = tf2::quatRotate(rot1, v_p_unit);
        tf2::Vector3 CW_vec = tf2::quatRotate(rot2, v_p_unit);
        
        pcl::PointXYZ v_intermediate_CCW( NODE_DIST * CCW_vec.x() ,  NODE_DIST * CCW_vec.y() , 0);
        pcl::PointXYZ v_intermediate_CW( NODE_DIST * CW_vec.x() ,  NODE_DIST * CW_vec.y() , 0);
        
        
        //std::cout << "Score " << score << " " << total <<std::endl;
        pcl::PointXYZ zeroPoint;
        
        float score1 = getSectionFreenessScore(distances , v_intermediate_CCW);
        float score2 = getSectionFreenessScore(distances , v_intermediate_CW);
        // std::cout << i <<  score1<< score2 << std::endl;
        //std::cout << CW_vec.x() <<  " " << CW_vec.y() << " " << score1 << std::endl; 
        //std::cout << CCW_vec.x() <<  " " << CCW_vec.y() << " " << score2 << std::endl; 
        if(score1 > 0.95){
            bestPoint =  v_intermediate_CCW;
            p.x = bestPoint.x;
            p.y = bestPoint.y;
            bestScore = score1;
            break;
        }
        if(score2 > 0.95){
            bestPoint =  v_intermediate_CW;
            bestScore = score2;
            p.x = bestPoint.x;
            p.y = bestPoint.y;

            break;
        }
        
        if(score1 > bestScore){
            bestPoint = v_intermediate_CCW;
            bestScore = score1;
            
        }
        if(score2 > bestScore){
            bestPoint = v_intermediate_CW;
            bestScore = score2;
        }
    
    }
    // std::cout <<"BEst Point" << bestPoint.x << " " << bestPoint.y << " " << bestPoint.z << " " << bestScore << std::endl;
    // if(bestScore < 0.4){
    //   return pcl::PointXYZ(0,0,0);
    // }

    if(bestPoint.x == 0 && bestPoint.y == 0 && bestPoint.z == 0){
        p.x = -1;
        p.y = 0;
    }
    
}

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

   size_t cloudSize = out.size();
  int count = 0;
  // extract valid points from input cloud
  float _factor = (nScanRings - 1) / (_upperBound - _lowerBound);

  std::vector<int> ringCount (16,0); 

   
  //  tf::StampedTransform transform;
  //   tf::TransformListener* listener = new tf::TransformListener();
  // try{
  //   listener->waitForTransform("/world", "/X1", ros::Time(0), ros::Duration(3.0));
  //   listener->lookupTransform( "/world", "/X1", ros::Time(0), transform);
  // }
  //  catch(tf::TransformException& ex){
  //   ROS_ERROR("Received an exception trying to transform a point from \"world\" to \"X1\": %s", ex.what());
    
  // }

  std::vector<std::pair<float , pcl::PointXYZ>> yawPoints;
  std::vector<std::tuple<float , float>> distances;
  
  float data[15][10000][3];

  for (int i=0; i<15; i++)
      for (int j=0; j<10000; j++)
	      for(int k = 0; k  < 3; k ++)
             data[i][j][k] = 0;

  for (int i = 0; i < cloudSize; i++) {

    pcl::PointXYZI point;
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

    float angle = std::atan(point.z / std::sqrt(point.y * point.y + point.x * point.x));
    int ring = int(((angle * 180.0 / M_PI) - _lowerBound) * _factor + 0.5);

    if(ring>=15){
      continue;
    }

    int colVal = ringCount[ring];
    if(colVal >= 10000){
      continue;
    }
    
    data[ring][colVal][0] = point.x;
    data[ring][colVal][1] = point.y;
    data[ring][colVal][2] = point.z;

    ringCount[ring] += 1;

  }
  
  out.clear();
  
  float maxDist = 0;
  float maxDistAngle = 0;
  
  pcl::PointXYZ pastPoint;
  // std::vector<std::tuple<float,float>>distances;

  for(int i =0 ; i < 10000;i += 1){
    pcl::PointXYZ point;
    getMaxTraversablePoint(data , i , &point);
    yawPoints.push_back(std::pair <float , pcl::PointXYZ> (getYaw(point) , point));
    distances.push_back(std::tuple <float , float> (getYaw(point) , getDistanceFromOrigin(point)));
  }

  std::sort(yawPoints.begin() , yawPoints.end() , sortFunc );
  std::sort(distances.begin() , distances.end() , sortFunc2);

pcl::PointXYZ p;
  getBestNextNode(distances , p );

  if((ros::Time::now() - last_goal) > ros::Duration(3,0)){
    ROS_INFO("Published Goal")
    last_goal = ros::Time::now();
    transformPointToWorldAndPublish(p);
  }
  



  // std::cout << "Next" << p.x << " " << p.y << std::endl;
 
    
  for(int i =0 ; i < yawPoints.size();i += 1){
    pcl::PointXYZ point = yawPoints[i].second;
    if(i == 0){
        pastPoint.x = point.x;
        pastPoint.y = point.y;
        pastPoint.z = point.z;    
    }

    float distBetPoint = getDistanceBetween(point, pastPoint);



  

    // while(distBetPoint > 0.1){
    //     pastPoint.x += (point.x - pastPoint.x)/std::fabs(point.x - pastPoint.x)*0.05;
    //     pastPoint.y += (point.y - pastPoint.y)/std::fabs(point.y - pastPoint.y)*0.05;
    //     pastPoint.z += 0;
    //     distBetPoint = getDistanceBetween(point, pastPoint);
    //     // std::cout << distBetPoint << std::endl;
    //     out.push_back(pastPoint);
    // }
    // auto pt = transformPointToWorld(point , listener);

    out.push_back(point);
    // out2.push_back(pt);

    pastPoint.x = point.x;
    pastPoint.y = point.y;
    pastPoint.z = point.z;  

  }
  

  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2 (out ,pc2);
  // pcl::PCLPointCloud2 pc3;
  // pcl::toPCLPointCloud2 (out2 ,pc3);

  // pc3.header.frame_id = "world";
  pub.publish(pc2);
  // pub2.publish(pc3);
  // pub_vel.publish(t);

 out.clear();
//  out2.clear();
  // pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  // pcl::fromROSMsg(*msg, laserCloudIn);
  // pub.publish(laserCloudIn);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;


  std::printf("subscriebrs ");
  ros::Subscriber sub = nh.subscribe<PointCloud>("lidar_input", 1, callback);
  // ros::Subscriber sub2 = nh.subscribe("goal_to_explore", 1, goalCallBack);
  std::printf("subscriebrs ");
  pub = nh.advertise<PointCloud> ("X1/points2", 1);
  pub3 = nh.advertise<geometry_msgs::PointStamped>("goal_to_explore", 1);
  // pub2 = nh.advertise<PointCloud> ("X1/points3", 1);
  // pub_vel = nh.advertise<geometry_msgs::Twist> ("X1/cmd_vel", 1);
  ros::spin();
}


