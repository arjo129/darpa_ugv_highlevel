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
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <vector>

#define _lowerBound -15.0
#define _upperBound 15.0
#define nScanRings 16

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;
ros::Publisher pub_vel;
pcl::PointCloud<pcl::PointXYZ> out;



int getRowVal(int angle_seg, int ring){
  return ring*40+angle_seg;
}

float getDistanceBetween(pcl::PointXYZ p1 , pcl::PointXYZ p2){
    return std::sqrt( (p2.z- p1.z) * (p2.z- p1.z) + (p2.y - p1.y) * (p2.y- p1.y) + (p2.x - p1.x) * (p2.x - p1.x) );
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


  if(angle > 0.52 || distanceFromOrigin(p2 , true) < distanceFromOrigin(p1 ,true)  ||(p2[2]- p1[2]) < -0.3){
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

void callback(const PointCloud::ConstPtr& msg)
{
  geometry_msgs::Transform trans;
  trans.translation.x = 0;
  trans.translation.y = 0;
  trans.translation.z = 0;
  trans.rotation.x = 0;
  trans.rotation.y = 0;
  trans.rotation.z = 0;
  trans.rotation.w = 0;
  pcl_ros::transformPointCloud	(*msg , out , trans);

   size_t cloudSize = out.size();
  int count = 0;
  // extract valid points from input cloud
  float _factor = (nScanRings - 1) / (_upperBound - _lowerBound);

  std::vector<int> ringCount (16,0); 
  
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

    // // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    count ++;

    float angle = std::atan(point.z / std::sqrt(point.y * point.y + point.x * point.x));
    int ring = int(((angle * 180.0 / M_PI) - _lowerBound) * _factor + 0.5);

    if(ring>=15){
      continue;
    }
	
    /*
    float flat_angle = std::atan(point.y/point.x);
    if(point.y > 0 && point.x > 0){
      flat_angle += M_PI;
    }else if(point.y > 0 && point.x < 0){
      flat_angle += 2 * M_PI;
    }else if(point.y < 0 && point.x < 0){
      
    }else if(point.y < 0 && point.x > 0){
      flat_angle += M_PI;
    }
     */
    //int flat_angle_Seg = flat_angle/(2*M_PI)*40;
    

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
  std::vector<std::tuple<float,float>>distances;

  for(int i =0 ; i < 10000;i += 1){
    pcl::PointXYZ point;
    getMaxTraversablePoint(data , i , &point);
    
    float dist = std::sqrt(point.x*point.x+point.y*point.y);
    float flat_angle = std::atan(point.y/point.x);
    
    if(i == 0){
        pastPoint.x = point.x;
        pastPoint.y = point.y;
        pastPoint.z = point.z;    
    }
    
    if(point.y > 0 && point.x > 0){
      flat_angle += M_PI;
    }else if(point.y > 0 && point.x < 0){
      flat_angle += 2 * M_PI;
    }else if(point.y < 0 && point.x < 0){
      
    }else if(point.y < 0 && point.x > 0){
      flat_angle += M_PI;
    }
    if(flat_angle > 2*M_PI/3 && flat_angle < 4*M_PI/3 ){
      flat_angle -= (M_PI);
      distances.push_back(std::tuple<float,float>(flat_angle,dist));
     
    }
    float distBetPoint = getDistanceBetween(point, pastPoint);

  

    while(distBetPoint > 0.1 && distBetPoint < 0.5){
        pastPoint.x += (point.x - pastPoint.x)/std::fabs(point.x - pastPoint.x)*0.1;
        pastPoint.y += (point.y - pastPoint.y)/std::fabs(point.y - pastPoint.y)*0.1;
        pastPoint.z += (point.z - pastPoint.z)/std::fabs(point.z - pastPoint.z)*0.1;
        distBetPoint = getDistanceBetween(point, pastPoint);
        out.push_back(pastPoint);
    }


    out.push_back(point);

    pastPoint.x = point.x;
    pastPoint.y = point.y;
    pastPoint.z = point.z;  

  }
  
  std::sort(distances.begin() , distances.end());

 
  std::vector<std::tuple<float,float>>freeSpaces;

  std::tuple<float,float> tempFree;
  std::get<0>(tempFree) = 0;
  std::get<1>(tempFree) = 0;

  bool Found = false;
  
  for(auto &a : distances){
    if(std::get<1>(a) > 4.0){
      if(!Found){
        std::get<0>(tempFree) = std::get<0>(a);
        Found = true;
      }
    }else{
      if(Found){
        std::get<1>(tempFree) = std::get<0>(a);
        freeSpaces.push_back(tempFree);
        Found = false;
      }
    }
  }

  if(Found){
    std::get<1>(tempFree) = M_PI/3;
    freeSpaces.push_back(tempFree);
  }

  std::vector<std::tuple<float,float>> frontiers;

  int frontierCount = 0;
  for(auto &a:freeSpaces){
      //printf("%f\t%f\t\t" , std::get<0>(a) ,std::get<1>(a));
     if(std::fabs(std::get<1>(a) - std::get<0>(a)) > 0.3){
      frontiers.push_back(std::tuple<float,float>(std::fabs(std::get<1>(a) - std::get<0>(a)) ,(std::get<1>(a) + std::get<0>(a))/2.0))  ;
      frontierCount++;
     }
  }
  
 /*
  for(int i =0 ; i < 1000;i += 1){
    maxDist = maxDist + std::get<1>(distances[i]);
  }
  maxDistAngle = std::get<0>(distances[250]);
  float distTemp = maxDistAngle;
  
  for(int i =500 ; i < distances.size()-500;i += 1){
    distTemp = distTemp - std::get<1>(distances[i-500]);
    distTemp = distTemp + std::get<1>(distances[i+500]);
    if(distTemp > maxDist){
      maxDist = distTemp;
      maxDistAngle = std::get<0>(distances[i]);
    }
  }
  */
  geometry_msgs::Twist t;

  
  
  
  std::sort(frontiers.begin() , frontiers.end());

  float angleToTurn = 0;
 
  if(frontiers.size() > 0){
    angleToTurn = std::get<1>(frontiers[frontiers.size()-1]);
   printf("%f Found\n" , angleToTurn);
  }
  
  
  if(frontiers.size() == 0){
    t.linear.x = 0;
    t.angular.z = 0.5;
  }else if(angleToTurn < M_PI/10 && angleToTurn > -M_PI/10){
    t.linear.x = 0.5;
  }else if(angleToTurn < 0){
    t.angular.z = angleToTurn/3;
    t.linear.x = 0.3;
  }else if(angleToTurn > 0){
    t.angular.z = angleToTurn/3;
    t.linear.x = 0.3;
  }


  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2 (out ,pc2);
  pub.publish(pc2);
  pub_vel.publish(t);

 out.clear();
  // pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  // pcl::fromROSMsg(*msg, laserCloudIn);
  // pub.publish(laserCloudIn);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("X1/points", 1, callback);
  pub = nh.advertise<PointCloud> ("X1/points2", 1);
  pub_vel = nh.advertise<geometry_msgs::Twist> ("X1/cmd_vel", 1);
  ros::spin();
}


