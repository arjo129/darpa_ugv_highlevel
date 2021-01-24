#include <ugv_groundmap/ugv_groundmap.h>


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

  // pcl::PointXYZ p;
  // getBestNextNode(distances , p );

  // if((ros::Time::now() - last_goal) > ros::Duration(3,0)){
  //   ROS_INFO("Published Goal");
  //   last_goal = ros::Time::now();
  //   transformPointToWorldAndPublish(p);
  // }
  

    
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

    pastPoint.x = point.x;
    pastPoint.y = point.y;
    pastPoint.z = point.z;  

  }
  

  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2 (out ,pc2);
  pub.publish(pc2);
  out.clear();
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;


  std::printf("subscriebrs ");
  ros::Subscriber sub = nh.subscribe<PointCloud>("lidar_input", 1, callback);
  std::printf("subscriebrs ");
  pub = nh.advertise<PointCloud> ("X1/points2", 1);
  // pub3 = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 1);
  ros::spin();
}


