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
#include <queue>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <graph_msgs/Edges.h>
#include <graph_msgs/GeometryGraph.h>
#include <tf2/convert.h>
#include <tf/transform_listener.h>
#include <lidar_frontier3d/frontier_request.h>

#define _lowerBound -15.0
#define _upperBound 15.0
#define nScanRings 16
#define NODE_DIST 2
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::PointCloud<pcl::PointXYZ> out;
graph_msgs::GeometryGraph graph;
ros::Publisher pub;
ros::Publisher pub2;

std::deque<pcl::PointXYZ> allUsedFrontiers;

int globalGraphId = 0;
std::string robot_name;

int global_i = 0;
geometry_msgs::Vector3 goal;
geometry_msgs::Vector3 current;
geometry_msgs::Quaternion currentPose;
geometry_msgs::Quaternion invPose;
float yaw_to_goal = 0;
int direction = -1;

std::map<int , pcl::PointXYZ > discreteMap;
    
std::map<int , std::set<int>> terrainGraph;

visualization_msgs::MarkerArray ma;

visualization_msgs::Marker marker;

std::vector<std::tuple<float,float>>distances;

int id = 0;
int mColor = 0;
bool done = true;

bool findNewFrontiers = false;
void processFrontiers(const sensor_msgs::PointCloud2& msg);
void processFrontierPointCloud(pcl::PointCloud<pcl::PointXYZ> &out2 , tf::TransformListener* listener);

float getDistanceBetween(pcl::PointXYZ p1 , pcl::PointXYZ p2){
    return std::sqrt( (p2.z- p1.z) * (p2.z- p1.z) + (p2.y - p1.y) * (p2.y- p1.y) + (p2.x - p1.x) * (p2.x - p1.x) );
}


float getDistanceFromOrigin(pcl::PointXYZ p){
    return std::sqrt( (p.z) * (p.z) + (p.y) * (p.y) + (p.x) * (p.x) );
}

float getDistanceFromRobot(pcl::PointXYZ p){
    return std::sqrt( (p.z - current.z) * (p.z - current.z) + (p.y - current.y) * (p.y - current.y) + (p.x - current.x) * (p.x - current.x) );
}

float hieuristic(pcl::PointXYZ p){
    for(auto a :allUsedFrontiers){
      if(getDistanceBetween(a , p) < 5.00){
        std::cout << "Somewhere i have already been too" <<std::endl;
        return 0;
      }
    }
    return std::pow(getDistanceFromOrigin(p),2);
}



float distanceFromOrigin(float *p, bool plane){

  if(plane){
    return std::sqrt((p[0]*p[0]) + (p[1]*p[1]));
  }else{
    return std::sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]));
  }
}


float getMagnitude(pcl::PointXYZ p1){
    return std::sqrt( p1.z * p1.z + p1.y * p1.y + p1.x * p1.x);
}



int getRowVal(int angle_seg, int ring){
  return ring*40+angle_seg;
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

int getArcSegmentFromYaw(float yaw , int numOfSegments){

  if(yaw < -M_PI/2){
    yaw += 5 * M_PI /2;
  }else{
    yaw += M_PI/2;
  }
  yaw = M_PI - yaw;
  if(yaw < 0){
    yaw += 2* M_PI;
  }
  // std::cout << "Arcyaw: " << yaw <<std::endl;

  return yaw / ((2*M_PI)/(float)numOfSegments);


}

int getSegmentValue(pcl::PointXYZ &point){

    int semiCircle = getDistanceFromOrigin(point)/2.0;
    int numSegmentsInSemi = ((semiCircle+1) * 2 + 1)*2;
    int arcSegment = getArcSegmentFromYaw(getYaw(point) , numSegmentsInSemi );
    
    // std::cout << semiCircle << " " << numSegmentsInSemi << " " << arcSegment << std::endl;  
    
    return 2*(semiCircle+1)*(semiCircle+1) - 2 + arcSegment;
}

float navYawFromSegmentYaw(float yaw){
  if(yaw > 3*M_PI/2){
    yaw -= 5 * M_PI /2;
  }else{
    yaw -= M_PI/2;
  }

  return yaw;

}

pcl::PointXYZ getPointFromLaserScan(float yaw, float distance){
    float firstQuadYaw = std::fabs(yaw) > M_PI/2 ? (M_PI - std::fabs(yaw)) : std::fabs(yaw);
    float x = distance* std::cos(firstQuadYaw);
    float y = distance* std::sin(firstQuadYaw);

    y = yaw > 0 ? y*-1 : y;
    x = std::fabs(yaw) > M_PI/2 ? x*-1 : x;

    pcl::PointXYZ point;

    point.x = x;
    point.y = y;
    point.z = 0;

    return point;

}

pcl::PointXYZ getCenterForSegment(std::vector<std::tuple<float,float>> &distances , int segment){

    int semiCircle = std::sqrt((segment/2+1)) -1;
    int numSegmentsInSemi = ((semiCircle+1) * 2 + 1)*2;
    int arcSegment = segment - (2*(semiCircle+1)*(semiCircle+1) - 2);

    float lower_bound_dist = semiCircle*2;
    float upper_bound_dist = (semiCircle + 1)*2;

    float lower_bound_angle = navYawFromSegmentYaw((2*M_PI)/numSegmentsInSemi * arcSegment);
    float upper_bound_angle = navYawFromSegmentYaw((2*M_PI)/numSegmentsInSemi * (arcSegment + 1));
    
    if(lower_bound_angle > upper_bound_angle){
        float temp = lower_bound_angle;
        lower_bound_angle = upper_bound_angle;
        upper_bound_angle  = temp;
    }
  //TODO
    float score = 0;

    float totalCount = 0;

    float totalYaw = 0;
    float totalYawCount = 0;
    
    for(auto &a: distances){
        if(std::get<0>(a) < upper_bound_angle && std::get<0>(a) > lower_bound_angle){

	          totalCount += 1;
            if(std::get<1>(a) > upper_bound_dist ){
                score += (upper_bound_dist + lower_bound_dist)/2.0;
            }else if(std::get<1>(a) > lower_bound_dist ){
                score += (std::get<1>(a) + lower_bound_dist)/2.0;
            }else{
                score += lower_bound_dist;
            }

        }
    }

    float centerYaw = (upper_bound_angle + lower_bound_angle)/2.0;
    // float centerYaw = totalYaw/totalYawCount;

    if(std::abs(upper_bound_angle) > M_PI/2 && std::abs(lower_bound_angle) > M_PI/2 && centerYaw < 0.1 && centerYaw > -0.1){
      centerYaw = centerYaw + M_PI - 0.1;
    }
    float centerDistance = (score/totalCount);

    return getPointFromLaserScan(centerYaw , centerDistance);

}

float getSectionFreenessScore(std::vector<std::tuple<float,float>> &distances , pcl::PointXYZ p , pcl::PointXYZ origin){
    
    pcl::PointXYZ v_p;
    
    v_p.x = p.x - origin.x;
    v_p.y = p.y - origin.y;
    v_p.z = p.z - origin.z;

    float v_p_mag = getMagnitude(v_p);
    
    tf2::Vector3 v_p_unit(v_p.x / v_p_mag, v_p.y / v_p_mag,v_p.z / v_p_mag);



    float dist_to_point = getDistanceBetween(p , origin);
    float lower_bound_dist = dist_to_point - 1;
    float upper_bound_dist = dist_to_point + 1;

    tf2::Quaternion rotation1(0,0,0.706,0.709);
    tf2::Quaternion rotation2(0,0,-0.706,0.709);

    tf2::Vector3 left_vector = tf2::quatRotate(rotation1, v_p_unit);
    
    tf2::Vector3 right_vector = tf2::quatRotate(rotation2, v_p_unit);
    
    pcl::PointXYZ cw_corner;
    pcl::PointXYZ ccw_corner;
    
    ccw_corner.x = left_vector.x() + v_p.x;
    ccw_corner.y = left_vector.y() + v_p.y;
    ccw_corner.z = left_vector.z() + v_p.z;

    cw_corner.x = right_vector.x() + v_p.x;
    cw_corner.y = right_vector.y() + v_p.y;
    cw_corner.z = right_vector.z() + v_p.z;
    
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

void populateMap(std::map<int , pcl::PointXYZ> &discreteMap , int maxNumberSegments , std::vector<std::tuple<float,float>> &distances){
  
  
  for(int i = 0 ; i <= maxNumberSegments; i ++){
    // auto point = getCenterForSegment(distances , i);

    pcl::PointXYZ point(0,0,0);

    discreteMap[i] = point;
    // std::cout << i << " " << point.x << " " << point.y <<std::endl; 
   
  }
}

int getBestNextNode(std::vector<std::tuple<float,float>> &distances , pcl::PointXYZ &p , pcl::PointXYZ &origin , std::map<int , pcl::PointXYZ> &discreteSegmentMap){
    
    pcl::PointXYZ v_p;
    
    v_p.x = p.x - origin.x;
    v_p.y = p.y - origin.y;
    v_p.z = p.z - origin.z;

    float v_p_mag = getMagnitude(v_p);
    
    tf2::Vector3 v_p_unit(v_p.x / v_p_mag, v_p.y / v_p_mag, v_p.z / v_p_mag);

    tf2::Quaternion rot1;
    tf2::Quaternion rot2;
    
    float bestScore = 0;
    pcl::PointXYZ bestPoint;
    
    for(int i = 0; i <= 90 ;i += 10){
        
        rot1.setRPY(0 ,0 , i/180.0*M_PI);
        rot2.setRPY(0 ,0 , -i/180.0*M_PI);
        
        tf2::Vector3 CCW_vec = tf2::quatRotate(rot1, v_p_unit);
        tf2::Vector3 CW_vec = tf2::quatRotate(rot2, v_p_unit);
        
        pcl::PointXYZ v_intermediate_CCW(p.x - NODE_DIST * CCW_vec.x() , p.y - NODE_DIST * CCW_vec.y() , p.z - NODE_DIST * CCW_vec.z());
        pcl::PointXYZ v_intermediate_CW(p.x - NODE_DIST * CW_vec.x() , p.y - NODE_DIST * CW_vec.y() , p.z - NODE_DIST * CW_vec.z());
        
        
        //std::cout << "Score " << score << " " << total <<std::endl;
        
        float score1 = getSectionFreenessScore(distances , v_intermediate_CCW , origin);
        float score2 = getSectionFreenessScore(distances , v_intermediate_CW , origin);
        // std::cout << i <<  score1<< score2 << std::endl;
        //std::cout << CW_vec.x() <<  " " << CW_vec.y() << " " << score1 << std::endl; 
        //std::cout << CCW_vec.x() <<  " " << CCW_vec.y() << " " << score2 << std::endl; 
        if(score1 > 0.95){
            bestPoint =  v_intermediate_CCW;
            bestScore = score1;
            break;
        }
        if(score2 > 0.95){
            bestPoint =  v_intermediate_CW;
            bestScore = score2;
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
    // std::cout << bestPoint.x << " " << bestPoint.y << " " << bestPoint.z << " " << bestScore << std::endl;
    // if(bestScore < 0.4){
    //   return pcl::PointXYZ(0,0,0);
    // }

    int segForPoint = getSegmentValue(p);
    int seg = getSegmentValue(bestPoint);

     
    bestPoint.z = 0;
    // std::cout << "segemnt: "<<segForPoint << std::endl;

    if(discreteSegmentMap.find(seg) == discreteSegmentMap.end()){

      // std::cout << "Not Found" << std::endl; 

      return 1;
      //No path

      
    }else{
      if(getDistanceFromOrigin(bestPoint) < 2.0){
       seg = -1;
      }

      if(seg == segForPoint){
        p.x = bestPoint.x;
        p.y = bestPoint.y;
        p.z = bestPoint.z;
        return 0;

      }

      terrainGraph[seg].insert(segForPoint);
      // std::cout << "Found" << std::endl; 
      auto pt = discreteSegmentMap[seg];

      if(pt.x == 0 && pt.y ==0 && pt.z == 0){
        discreteSegmentMap[seg] = bestPoint;
        p.x = bestPoint.x;
        p.y = bestPoint.y;
        p.z = bestPoint.z;
        return 0;
        //continue finding to startpoint
      }
      p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
      return 0; // path alr found
      

    }
    

}


void positionCallBack(const nav_msgs::Odometry::ConstPtr& msg){
    nav_msgs::Odometry poseMsg = *msg;
    
    current.x = poseMsg.pose.pose.position.x;
    current.y = poseMsg.pose.pose.position.y;
    current.z = poseMsg.pose.pose.position.z;
    
    currentPose.x = poseMsg.pose.pose.orientation.x;
    currentPose.y = poseMsg.pose.pose.orientation.y;
    currentPose.z = poseMsg.pose.pose.orientation.z;
    currentPose.w = poseMsg.pose.pose.orientation.w;

    invPose.x = poseMsg.pose.pose.orientation.x;
    invPose.y = poseMsg.pose.pose.orientation.y;
    invPose.z = poseMsg.pose.pose.orientation.z;
    invPose.w = -poseMsg.pose.pose.orientation.w;
    
        
 
}


void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){

  findNewFrontiers = true;

  //TO enable Filtered Frontiers use the below

  // ros::NodeHandle n;
  // ros::ServiceClient client = n.serviceClient<lidar_frontier3d::frontier_request>("get_frontiers");
  // lidar_frontier3d::frontier_request srv;
   
  // if (client.call(srv)){
  //   ROS_INFO("received frontiers");
  //   auto frontiers = srv.response.points;
    
  //   processFrontiers(frontiers);
  // }else{
  //   ROS_ERROR("Failed to call service add_two_ints");
  // }

  
}

pcl::PointXYZ transformPointToWorld(pcl::PointXYZ point , tf::TransformListener* listener){

  geometry_msgs::PointStamped initial_pt; 
  initial_pt.header.frame_id = robot_name+"/base_link";
  initial_pt.header.stamp = ros::Time::now();
  initial_pt.point.x = point.x;
  initial_pt.point.y = point.y;
  initial_pt.point.z = point.z;
  
  geometry_msgs::PointStamped transformStamped;
  pcl::PointXYZ transformedPoint;
  try{
    listener->waitForTransform(robot_name+"/world", initial_pt.header.frame_id, initial_pt.header.stamp, ros::Duration(3.0));
    listener->transformPoint(robot_name+"/world", initial_pt , transformStamped);	
  } catch(tf::LookupException ex) {
    ROS_ERROR("Lookup exception occurred %s", ex.what());
  }
  transformedPoint.x = transformStamped.point.x;
  transformedPoint.y = transformStamped.point.y;
  transformedPoint.z = transformStamped.point.z;

  return transformedPoint;

}

void processFrontiers(const sensor_msgs::PointCloud2& msg){

  std::cout << "Processing Frontiers " << msg.header << std::endl;

  pcl::PointCloud<pcl::PointXYZ> out2;
  
  // if(findNewFrontiers){

  populateMap(discreteMap , 1000 , distances);


  findNewFrontiers = false;
  tf::TransformListener* listener = new tf::TransformListener();
   tf::StampedTransform transform;
  try{
    listener->waitForTransform(robot_name+"/world", msg.header.frame_id, msg.header.stamp, ros::Duration(3.0));
    listener->lookupTransform( robot_name+"/world", msg.header.frame_id, msg.header.stamp, transform);
  }
   catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"world\" to \"X1\": %s", ex.what());
    return;
  }

  pcl::fromROSMsg(msg, out2);
  processFrontierPointCloud(out2, listener); 

}

void frontierCallBack(const PointCloud::ConstPtr& msg){

  pcl::PointCloud<pcl::PointXYZ> out2;
  
  if(findNewFrontiers){

    ROS_INFO( "Need New Frontier");

    findNewFrontiers = false;


  populateMap(discreteMap , 1000 , distances);


  findNewFrontiers = false;
  tf::TransformListener* listener = new tf::TransformListener();
   tf::StampedTransform transform;
  try{
    listener->waitForTransform(robot_name+"/world", msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
    listener->lookupTransform(robot_name+"/world", msg->header.frame_id, ros::Time(0), transform);
  }
   catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"world\" to \"X1\": %s", ex.what());
    return;
  }
  pcl_ros::transformPointCloud	(*msg , out2 , transform);
  processFrontierPointCloud(out2 , listener);

  }

}
bool Compare(pcl::PointXYZ left, pcl::PointXYZ right)
{
    return getDistanceFromRobot(left) < getDistanceFromRobot(right);
}



template<typename T> void getFinalPathToGoal(std::deque<int> &finalPathToFrontier , tf::TransformListener* listener , T& allFrontiers){

  bool firstTimeHEre = true;
  bool foundOnePoint = false;
  while(!allFrontiers.empty()) {
    ROS_INFO("F Popping");
    pcl::PointXYZ point = allFrontiers.top();
    allFrontiers.pop();
    point.z = 0;

    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    if(firstTimeHEre){
      firstTimeHEre = false;
      marker.color.r = 1.0;
       marker.color.g = 1.0;
       marker.color.b = 1.0;

    }else{
      marker.color.r = 0.0;
       marker.color.g = 1.0;
       marker.color.b = 0.0;
    }

    marker.color.a = 1.0;
    
       marker.pose.orientation.w = 1.0;
       marker.pose.position.x = point.x;
       marker.pose.position.y = point.y;
       marker.pose.position.z = point.z;
        marker.id = ++id;
       ma.markers.push_back(marker);

    geometry_msgs::Vector3 currentC = current;
    geometry_msgs::Quaternion currentPoseC = currentPose;
    geometry_msgs::Quaternion invPoseC = invPose;


    auto now = ros::Time::now();

    geometry_msgs::PointStamped initial_pt; 
    initial_pt.header.frame_id = robot_name+"/world";
    initial_pt.header.stamp = now;
    initial_pt.point.x = point.x;
    initial_pt.point.y = point.y;
    initial_pt.point.z = point.z;
    
    geometry_msgs::PointStamped transformStamped;
    try{
      listener->waitForTransform(robot_name+"/base_link", initial_pt.header.frame_id, initial_pt.header.stamp, ros::Duration(1.0));
    listener->transformPoint(robot_name+"/base_link", initial_pt , transformStamped);	
     goal.x = transformStamped.point.x;
      goal.y = transformStamped.point.y;
      goal.z = transformStamped.point.z;
    } catch(tf::LookupException e) {
      ROS_ERROR("Failed to transform : %s. Lone: %d", e.what(), __LINE__);
      return;
    }

    mColor++;


    pcl::PointXYZ middle_point;
    middle_point.x = goal.x;
    middle_point.y = goal.y;
    middle_point.z = 0;
      pcl::PointXYZ start_point;
      start_point.x = 0;
      start_point.y = 0;
      start_point.z = 0;




      std::deque<int> singlePathToFrontier;

      
    int countPath = 0;
    int stateOfPathFinding = 0;
      while(getDistanceBetween(middle_point , start_point) > 3 && stateOfPathFinding == 0){
        stateOfPathFinding = getBestNextNode(distances , middle_point , start_point, discreteMap);
        std::cout << stateOfPathFinding << std::endl;
        // if(stateOfPathFinding == 2)break;
        singlePathToFrontier.push_front(getSegmentValue(middle_point));

        if(getDistanceBetween(middle_point , start_point) <= 3){
          terrainGraph[-1].insert(getSegmentValue(middle_point));
          std::cout << "Ending" <<std::endl;
          foundOnePoint = true;
          allUsedFrontiers.push_back(point);
          break;
        }
      
        geometry_msgs::PointStamped inputMPoint;

        // inputMPoint.header.stamp = ros::Time::now().toNSec();
        inputMPoint.header.frame_id = robot_name+"/base_link";
        inputMPoint.header.stamp = now;
        inputMPoint.point.x = middle_point.x;
        inputMPoint.point.y = middle_point.y;
        inputMPoint.point.z = middle_point.z;

        ROS_INFO( "MIDDLE POINT %f %f %f" , middle_point.x, middle_point.y, middle_point.z);
// 
        geometry_msgs::PointStamped stamped_out;
        try{
          listener->waitForTransform(robot_name+"/world", inputMPoint.header.frame_id , now, ros::Duration(3.0));
          listener->transformPoint(robot_name+"/world", inputMPoint , stamped_out);	
        } catch (tf::LookupException e) {
          ROS_ERROR("%s:%d: %s", __FILE__,  __LINE__, e.what());
        }
        marker.color.r = 1.0;
       marker.color.g = 0.0;
       marker.color.b = 0.0;
      marker.pose.position.x = stamped_out.point.x;
       marker.pose.position.y = stamped_out.point.y;
       marker.pose.position.z = 0;
             

       id++;
       marker.id = id;
       ma.markers.push_back(marker);
        countPath++;
        if(countPath > 30)break;
      }

      if(foundOnePoint){
        finalPathToFrontier = singlePathToFrontier;
        break;
      }

    }


}



void processFrontierPointCloud(pcl::PointCloud<pcl::PointXYZ> &out2, tf::TransformListener* listener){

  size_t cloudSize = out2.size();
  ROS_INFO("Frontiers: %lu", cloudSize);
  int countPath = 0;
  auto cmp = [](pcl::PointXYZ left, pcl::PointXYZ right) { return hieuristic(left) < hieuristic(right); };
  std::priority_queue<pcl::PointXYZ , std::deque<pcl::PointXYZ> , decltype(cmp)> allFrontiers(cmp);

  for (int i = 0; i < cloudSize; i++) {

    pcl::PointXYZ point;
    point.x = out2[i].x;
    point.y = out2[i].y;
    point.z = out2[i].z;

    ROS_INFO("Possible Frontiers %f %f %f", point.x , point.y, point.z);

    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001 && point.z > 2) {
      continue;
    }
    if(point.x < 0) continue;

    allFrontiers.push(point);

  }
  std::deque<int> finalPathToFrontierC;
  getFinalPathToGoal(finalPathToFrontierC , listener , allFrontiers);

   ROS_INFO("I am here");

   std::deque<int> finalPathToFrontier;

    for(int a = 0; a < finalPathToFrontierC.size()-1 ; a++){
      if(finalPathToFrontierC[a] >= 6){
	      finalPathToFrontier.push_back(finalPathToFrontierC[a]);
      }
     }
    if(finalPathToFrontier[0] != -1){
      finalPathToFrontier.push_front(-1);
    }

    for(int a = 0; a < finalPathToFrontier.size() ; a++){
      ROS_INFO("Final frontier:%d", finalPathToFrontier[a]);
    }

    // std::map<int , std::set<int>> terrainGraphConnected;

    // std::queue<int> frontier;
    // frontier.push(-1);
    // while(!frontier.empty()){
    //   auto topVal = frontier.front();
    //   frontier.pop();
    //   if(terrainGraphConnected.find(topVal) != terrainGraphConnected.end())continue;
    //   for(auto a: terrainGraph[topVal]){
    //     terrainGraphConnected[topVal].insert(a);
    //     frontier.push(a);
    //   }
    // }

    // std::map<int , int> nodeIdMapping;
    graph_msgs::GeometryGraph gg;
    gg.header.seq = globalGraphId;
    gg.header.frame_id = robot_name+"/world";
    gg.header.stamp = ros::Time::now();
    int point_id = 0;
    std::cout << __FILE__ <<":" << __LINE__ << std::endl;
    for(int a = 0; a < finalPathToFrontier.size()-1 ; a++){
      if(finalPathToFrontier[a] == -1 ){
        geometry_msgs::Point tempPt;
        tempPt.x = current.x;
        tempPt.y = current.y;
        tempPt.z = current.z;
        gg.nodes.push_back(tempPt);
        gg.edges.push_back(graph_msgs::Edges());
        gg.edges[a].node_ids.push_back(a+1);
        
      }else{
        std::cout << __FILE__ <<":" << __LINE__ << std::endl;
        auto pt = transformPointToWorld(discreteMap[finalPathToFrontier[a]] , listener);
        std::cout << __FILE__ <<":" << __LINE__ << std::endl;
        geometry_msgs::Point tempPt;
        tempPt.x =pt.x;
        tempPt.y =pt.y;
        tempPt.z =pt.z;
        gg.nodes.push_back(tempPt);
        gg.edges.push_back(graph_msgs::Edges());
        gg.edges[a].node_ids.push_back(a+1);
      }
    }
std::cout << __FILE__ <<":" << __LINE__ << std::endl;
    auto pt = transformPointToWorld(discreteMap[finalPathToFrontier[finalPathToFrontier.size()-1]] , listener);
    geometry_msgs::Point tempPt;
    tempPt.x =pt.x;
    tempPt.y =pt.y;
    tempPt.z =pt.z;
    gg.nodes.push_back(tempPt);
    gg.edges.push_back(graph_msgs::Edges());
    
    for(auto a = 0; a < gg.nodes.size() ; a++){
      ROS_INFO( "Node: %d", a );
      for(auto b: gg.edges[a].node_ids){
        std::cout <<  b << " ";
      } 
      std::cout << std::endl;
    }

      pub.publish(ma);

      pub2.publish(gg);

      terrainGraph.clear();

    //TODO add logic here

    globalGraphId++;
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
  distances.clear();		

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

    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001 && point.z < 2) {
      continue;
    }

    count ++;

    float flat_angle = getYaw(point);
    float dist = std::sqrt(point.x*point.x+point.y*point.y);

	distances.push_back(std::tuple<float,float>(flat_angle,dist));

  }
  
  
  
  out.clear();
 

}

int main(int argc, char** argv)
{

  marker.header.frame_id = robot_name+"/world";
  marker.type = marker.SPHERE;
  marker.action = marker.ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<PointCloud>("traversable_pointcloud_input", 1, callback);
  ros::Subscriber sub3 = nh.subscribe("robot_position_pose", 1, positionCallBack);
  ros::Subscriber sub2 = nh.subscribe("goal_to_explore", 1, goalCallBack);
  ros::Subscriber sub4 = nh.subscribe<PointCloud>("frontiers", 1, frontierCallBack);
  pub = nh.advertise<visualization_msgs::MarkerArray> ("Path", 1);
  pub2 = nh.advertise<graph_msgs::GeometryGraph> ("graph", 1);
  ros::param::get("~robot_name", robot_name);
  ros::spin();
}


