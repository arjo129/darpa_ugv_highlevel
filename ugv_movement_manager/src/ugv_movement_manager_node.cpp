
#include <ugv_movement_manager/ugv_movement_manager.h>
#include <ugv_movement_manager/naive_strat.h>

#define _lowerBound -15.0
#define _upperBound 15.0
#define nScanRings 16





void positionCallBack(const nav_msgs::Odometry::ConstPtr& msg){
    

    nav_msgs::Odometry poseMsg = *msg;
    geometry_msgs::Quaternion currentPose;
    // std::cout << "Got position fix" <<std::endl;
    current.x = poseMsg.pose.pose.position.x;
    current.y = poseMsg.pose.pose.position.y;
    current.z = poseMsg.pose.pose.position.z;
    
    currentPose.x = poseMsg.pose.pose.orientation.x;
    currentPose.y = poseMsg.pose.pose.orientation.y;
    currentPose.z = poseMsg.pose.pose.orientation.z;
    currentPose.w = poseMsg.pose.pose.orientation.w;

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
    
    

}

void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
    current_state = VehicleState::MOVING;
    last_goal = ros::Time::now();
    geometry_msgs::PointStamped goalPosition = *msg;
    goal.x = goalPosition.point.x;
    goal.y = goalPosition.point.y;
    goal.z = 0;

    std::cout << "Got goal" << goal.x << ", " << goal.y <<std::endl;
}

void forcingGoal(pcl::PointXYZ goalPoint){
    current_state = VehicleState::MOVING;
    last_goal = ros::Time::now();
    goal.x = goalPoint.x;
    goal.y = goalPoint.y;
    goal.z = 0;

    std::cout << "Got Fake goal" << goal.x << ", " << goal.y <<std::endl;
}



void callback(const PointCloud::ConstPtr& msg){
  // std::cout << "Received Traversable Map" << std::endl;
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

   pcl::PointXYZ distantPoint;
   float maxDistSoFar = 0;

  for (int i = 0; i < out.size(); i++) {

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


    if(fabs(flat_angle) < M_PI/2 && maxDistSoFar < dist){
      distantPoint.x = point.x;
      distantPoint.y = point.y;
      distantPoint.z = point.z;
      maxDistSoFar = dist;
    }

	  distances.push_back(std::tuple<float,float>(flat_angle,dist));

  }
  
  std::sort(distances.begin() , distances.end());

  if(current_strategy == StategyState::NAIVE && current_state == VehicleState::AWAITING_INSTRUCTION){
    pcl::PointXYZ p;
    getBestNextNode(distances , p );

    p = transformPointToWorld(p);

    forcingGoal(p);
  }
  


  int HALT_STATE = haltOrCancelGoal(current_state , last_goal , goal , current);

  if(HALT_STATE != 0){
    std_msgs::Int8 i;
    current_state = VehicleState::AWAITING_INSTRUCTION; 
    if(HALT_STATE == 1) i.data = 0;
    else i.data = -1;
    status_pub.publish(i);
    geometry_msgs::Twist t;
    t.linear.x = 0;
    t.angular.z = 0;
    pub_vel.publish(t);
    return;
  }

  float v = getForwardVelocity(distances);
  float w = getAngularVelocity(distances);

  v = v*1.5;

  if(v > 2) v = 2;
  else if(v < -2) v = -2;

  if(w > 1.0) w = 1.0;
  else if(w < -1.0) w = -1.0;
  
  geometry_msgs::Twist t;

  t.linear.x = v;
  t.angular.z = w;

  pub_vel.publish(t);


}

void onToggleStratCallback(const std_msgs::Empty e){
  if(current_strategy == StategyState::NAIVE){
    current_strategy = StategyState::EXPLORATION;
    std::cout << current_strategy << std::endl;
  }else{
    current_strategy = StategyState::NAIVE;
    std::cout << current_strategy << std::endl;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("traversable_pointcloud_input", 1, callback);
  ros::Subscriber sub3 = nh.subscribe("robot_position_pose", 1, positionCallBack);
  ros::Subscriber sub2 = nh.subscribe("goal_to_explore", 1, goalCallBack);
  ros::Subscriber toggleState = nh.subscribe("toggle_strategy", 1, onToggleStratCallback);
  pub_vel = nh.advertise<geometry_msgs::Twist> ("X1/cmd_vel", 1);
  status_pub = nh.advertise<std_msgs::Int8> ("status", 1);
  ros::spin();
}
