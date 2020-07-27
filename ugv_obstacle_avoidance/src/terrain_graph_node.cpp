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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <graph_msgs/Edges.h>
#include <graph_msgs/GeometryGraph.h>

#define _lowerBound -15.0
#define _upperBound 15.0
#define nScanRings 16
#define NODE_DIST 2
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


graph_msgs::GeometryGraph graph;
ros::Publisher pub;
ros::Publisher pub_vel;
pcl::PointCloud<pcl::PointXYZ> out;

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



float getDistanceBetween(pcl::PointXYZ p1 , pcl::PointXYZ p2){
    return std::sqrt( (p2.z- p1.z) * (p2.z- p1.z) + (p2.y - p1.y) * (p2.y- p1.y) + (p2.x - p1.x) * (p2.x - p1.x) );
}


float getDistanceFromOrigin(pcl::PointXYZ p){
    return std::sqrt( (p.z) * (p.z) + (p.y) * (p.y) + (p.x) * (p.x) );
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
  std::cout << "Arcyaw: " << yaw <<std::endl;

  return yaw / ((2*M_PI)/(float)numOfSegments);


}

int getSegmentValue(pcl::PointXYZ &point){

    int semiCircle = getDistanceFromOrigin(point)/2.0;
    int numSegmentsInSemi = ((semiCircle+1) * 2 + 1)*2;
    int arcSegment = getArcSegmentFromYaw(getYaw(point) , numSegmentsInSemi );
    
    std::cout << semiCircle << " " << numSegmentsInSemi << " " << arcSegment << std::endl;  
    
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
    //std::cout << "Score " << score << " " << total <<std::endl;
    //std::cout << p.x <<  " " << p.y << " " << p.z << std::endl; 
    
    return score/total;
}

void populateMap(std::map<int , pcl::PointXYZ> &discreteMap , int maxNumberSegments , std::vector<std::tuple<float,float>> &distances){
  
  // visualization_msgs::Marker test;
  //     test.header.frame_id = "simple_cave_01";
  //     test.type = test.SPHERE;
  //      test.action = test.ADD;
  //      test.scale.x = 0.2;
  //      test.scale.y = 0.2;
  //      test.scale.z = 0.2;
  //      test.color.a = 1.0;
  //      test.color.r = 0.0;
  //      test.color.g = 1.0;
  //      test.color.b = 1.0;
  //      test.pose.orientation.w = 1.0;
       

    // for(auto a: discreteMap){
    //   std::cout << a.first << ": " << a.second.x << " " << a.second.y << " " <<  getDistanceFromOrigin(a.second)  << " " << getYaw(a.second) << std::endl;
    //    test.id = a.first + 1000*(id+1);
    //    test.pose.position.x = a.second.x;
    //    test.pose.position.y = a.second.y;
    //    test.pose.position.z = 0;
        
    //    ma.markers.push_back(test);
    // }
  
  for(int i = 0 ; i <= maxNumberSegments; i ++){
    // auto point = getCenterForSegment(distances , i);

    pcl::PointXYZ point(0,0,0);

    discreteMap[i] = point;
    std::cout << i << " " << point.x << " " << point.y <<std::endl; 
    // test.id = i + 1000*(id+1);
    // test.color.r  = ((float)i)/maxNumberSegments;
    // test.pose.position.x = point.x;
    // test.pose.position.y = point.y;
    // test.pose.position.z = point.z;
    // ma.markers.push_back(test);
  }
}

pcl::PointXYZ getBestNextNode(std::vector<std::tuple<float,float>> &distances , pcl::PointXYZ &p , pcl::PointXYZ &origin , std::map<int , pcl::PointXYZ> &discreteSegmentMap){
    
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
        
        std::cout << i << std::endl;
        //std::cout << "Score " << score << " " << total <<std::endl;
        
        float score1 = getSectionFreenessScore(distances , v_intermediate_CCW , origin);
        float score2 = getSectionFreenessScore(distances , v_intermediate_CW , origin);
        //std::cout << CW_vec.x() <<  " " << CW_vec.y() << " " << score1 << std::endl; 
        //std::cout << CCW_vec.x() <<  " " << CCW_vec.y() << " " << score2 << std::endl; 
        if(score1 > 0.95){
            bestPoint =  v_intermediate_CCW;
            break;
        }
        if(score2 > 0.95){
            bestPoint =  v_intermediate_CW;
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
    //for(auto a: discreteSegmentMap){
      //std::cout << "Finding node !!" << a.first << ": " << a.second.x << " " << a.second.y << " " <<  getDistanceFromOrigin(a.second)  << " " << getYaw(a.second) << std::endl;
    //}
    int segForPoint = getSegmentValue(p);
    int seg = getSegmentValue(bestPoint);

     
    
    std::cout << "segemnt: "<<segForPoint << std::endl;

    // marker.color.r = 1.0;
    //    marker.color.g = 0.0;
    //    marker.color.b = 0.0;
    //  marker.pose.position.x = bestPoint.x;
    //    marker.pose.position.y = bestPoint.y;
    //    marker.pose.position.z = bestPoint.z;
    //    id++;
    //    marker.id = id;
    //    ma.markers.push_back(marker);


    // p.x = bestPoint.x;
    // p.y = bestPoint.y;
    // p.z = bestPoint.z;
    terrainGraph[seg].insert(segForPoint);

    if(discreteSegmentMap.find(seg) == discreteSegmentMap.end()){

      std::cout << "Not Found" << std::endl; 

      return bestPoint;

      
    }else{

      std::cout << "Found" << std::endl; 
      auto pt = discreteSegmentMap[seg];

      if(pt.x == 0 && pt.y ==0 && pt.z == 0){
        discreteSegmentMap[seg] = bestPoint;
        return bestPoint;
      }

      return pt;
      

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
    
    /*
    
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
    
    */
    
    //std::printf("%f\n" , yaw_to_goal);
    
        
 
}

void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){


  //TODO after integrating arjo's point cloud package, loop through and for every point perform the equivalent of getting one frontier to gernerate local graph
  /*
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

    //TODO add logic here

  }
  */
  //TODO ends here



   /*std::string goalStr = msg->data.c_str();
   std::vector <std::string> tokens; 
      
    // stringstream class check1 
    std::stringstream check1(goalStr); 
      
    std::string intermediate; 
      
    // Tokenizing w.r.t. space ' ' 
    while(std::getline(check1, intermediate, ' ')) 
    { 
        tokens.push_back(intermediate); 
    } 
    goal.x = std::atof(tokens[0].c_str());
    goal.y = std::atof(tokens[1].c_str());
    goal.z = std::atof(tokens[2].c_str());*/

    geometry_msgs::PointStamped goalPosition = *msg;
    goal.x = goalPosition.point.x;
    goal.y = goalPosition.point.y;
    goal.z = goalPosition.point.z;


    geometry_msgs::Vector3 currentC = current;
    geometry_msgs::Quaternion currentPoseC = currentPose;
    geometry_msgs::Quaternion invPoseC = invPose;

    tf2::Quaternion rotation1(
    invPoseC.x,
    invPoseC.y,
    invPoseC.z,
    invPoseC.w);


    tf2::Vector3 vector(goal.x -currentC.x, goal.y - currentC.y, goal.z-currentC.z);
    tf2::Vector3 goal_new_coord = tf2::quatRotate(rotation1, vector);

    std::cout << goal_new_coord.x() <<  " " << goal_new_coord.y() << " " << goal_new_coord.z() << std::endl;

    mColor++;


    pcl::PointXYZ middle_point;
    middle_point.x = goal_new_coord.x();
    middle_point.y = goal_new_coord.y();
    middle_point.z = goal_new_coord.z();
    // float distToGoal = getDistanceFromOrigin(middle_point);
    // int maxNumberSegments = 2*(distToGoal/2 + 4)*(distToGoal/2 + 4);
    


      pcl::PointXYZ start_point;
      start_point.x = 0;
      start_point.y = 0;
      start_point.z = 0;
      
      marker.color.r = 1.0;
       marker.pose.orientation.w = 1.0;
       marker.pose.position.x = goal.x;
       marker.pose.position.y = goal.y;
       marker.pose.position.z = goal.z;
        marker.id = ++id;
       ma.markers.push_back(marker);
      
      std::cout << "Starting" << std::endl;
      std::cout << middle_point.x <<  " " << middle_point.y << " " << middle_point.z << std::endl; 

      int seg = getSegmentValue(middle_point);

      auto pt = discreteMap[seg];

      if(pt.x == 0 && pt.y ==0 && pt.z == 0){
        discreteMap[seg] = middle_point;
      }

      
    int count = 0;
      //TODO pass in position in X1 frame and not global frame and transform later.
      while(getDistanceBetween(middle_point , start_point) > 2){
        middle_point = getBestNextNode(distances , middle_point , start_point, discreteMap);
      
        pcl::PointXYZ marker_point  = middle_point;
        std::cout << "Mid";
        std::cout << middle_point.x <<  " " << middle_point.y << " " << middle_point.z << std::endl; 

        tf2::Vector3 globalCoordMarker(marker_point.x +currentC.x, marker_point.y + currentC.y, marker_point.z+currentC.z);
        tf2::Quaternion rotation2(
          currentPoseC.x,
          currentPoseC.y,
          currentPoseC.z,
          currentPoseC.w);
        tf2::Vector3 globalCoordMarkerWPose = tf2::quatRotate(rotation2, globalCoordMarker);

        marker.color.r = 0.0;
       marker.color.g = 0.0;
       marker.color.b = 1.0;
      marker.pose.position.x = globalCoordMarkerWPose.x();
       marker.pose.position.y = globalCoordMarkerWPose.y();
       marker.pose.position.z = globalCoordMarkerWPose.z();

       
       id++;
       marker.id = id;
       ma.markers.push_back(marker);

        count++;
        if(count > 20){
            break;
        }
      }


      std::cout << "Graph" << std::endl;
      for(auto a: terrainGraph){
        std::cout << "Node: " <<  a.first << std::endl;
        for(auto b : a.second){
          std::cout << b << " ";

        }
        std::cout << std::endl;
      }
      pub.publish(ma);
    
 

    
    //std::printf("%f\t%f\t%f\n" , goal.x, goal.y , goal.z);
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

    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
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

  marker.header.frame_id = "simple_cave_01";
marker.type = marker.SPHERE;
  marker.action = marker.ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  populateMap(discreteMap , 500 , distances);


  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  std::printf("subscriebrs ");
  ros::Subscriber sub = nh.subscribe<PointCloud>("traversable_pointcloud_input", 1, callback);
  ros::Subscriber sub3 = nh.subscribe("robot_position_pose", 1, positionCallBack);
  ros::Subscriber sub2 = nh.subscribe("goal_to_explore", 1, goalCallBack);
  pub = nh.advertise<visualization_msgs::MarkerArray> ("Path", 1);
  std::printf("subscriebrs ");
  ros::spin();
}


