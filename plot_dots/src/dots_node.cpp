/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <map_merge/laser_operations.h>

#include <iostream>
#include <list>
#include <unordered_map>
#include <tuple>
#include <queue> 

#include <boost/functional/hash.hpp>
#include <math.h> 

ros::Publisher pub;
const float dot_distance = 3;
const float EPSILON = 0.0001;

struct key_hash {
  std::size_t operator()(const std::tuple<float, float, float>& k) const {
      using boost::hash_value;
      using boost::hash_combine;

      // Start with a hash value of 0    .
      std::size_t seed = 0;

      //hash_combine(seed,hash_value(k.first));
	  //hash_combine(seed,hash_value(k.second));
      //hash_combine(seed,hash_value(k.third));
	  hash_combine(seed,hash_value(std::get<0>(k)));
	  hash_combine(seed,hash_value(std::get<1>(k)));
	  hash_combine(seed,hash_value(std::get<2>(k)));

      // Return the result.
      return seed;
  }
};

struct key_equal : public std::binary_function<std::tuple<float, float, float>, std::tuple<float, float, float>, bool> {
  bool operator()(const std::tuple<float, float, float>& v0, const std::tuple<float, float, float>& v1) const {
   return (
   fabs(std::get<0>(v0) - std::get<0>(v1)) < EPSILON &&
   fabs(std::get<1>(v0) - std::get<1>(v1)) < EPSILON &&
   fabs(std::get<2>(v0) - std::get<2>(v1)) < EPSILON
  );
  }
};


//standardise to making use of pcl::pointXYZ only instead of std::tuple

void add_to_queue(std::unordered_map<std::tuple<float, float, float>,bool,key_hash,key_equal>& visited_map, pcl::PointXYZ& point,std::queue<pcl::PointXYZ>& q, LidarScan& scan) 
{
	std::tuple<float, float, float> tempUp, tempDown, tempLeft, tempRight, tempForward, tempBackward;
	pcl::PointXYZ pointUp, pointDown, pointLeft, pointRight, pointForward, pointBackward;
	
	std::get<0>(tempUp) = point.x;
	std::get<1>(tempUp) = point.y;
	std::get<2>(tempUp) = point.z + dot_distance;
	pointUp.x = point.x;
	pointUp.y  = point.y;
	pointUp.z  = point.z + dot_distance;	
	auto search = visited_map.find(tempUp);
	if (search == visited_map.end() && isPointInside(scan, pointUp)) { //did NOT manage to find it in the map, therefore is NOT visited
		std::cout << "pushed into queue Down: " << point.x << ", " << point.y << ", " << point.z + dot_distance << std::endl;	
		q.push(pointUp); //add it to the queue
		visited_map.insert({tempUp, true}); //add it to visited points
	} else {
		//std::cout << "FOUND INSIDE OF MAP" << std::endl;
	}

	
	std::get<0>(tempDown) = point.x;
	std::get<1>(tempDown) = point.y;
	std::get<2>(tempDown) = point.z - dot_distance;
	pointDown.x = point.x;
	pointDown.y  = point.y;
	pointDown.z  = point.z - dot_distance;
	search = visited_map.find(tempDown);
	if (search == visited_map.end() && isPointInside(scan, pointDown)) { //did not manage to find it in the map, therefore is not visited
		std::cout << "pushed into queue Down: " << point.x << ", " << point.y << ", " << point.z - dot_distance << std::endl;	
		q.push(pointDown); //add it to the queue
		visited_map.insert({tempDown, true}); //add it to visited points
	}  else {
		//std::cout << "FOUND INSIDE OF MAP" << std::endl;
	}

	
	std::get<0>(tempLeft) = point.x;
	std::get<1>(tempLeft) = point.y - dot_distance;
	std::get<2>(tempLeft) = point.z;
	pointLeft.x = point.x;
	pointLeft.y  = point.y - dot_distance;
	pointLeft.z  = point.z;	
	search = visited_map.find(tempLeft);
	if (search == visited_map.end() && isPointInside(scan, pointLeft)) { //did not manage to find it in the map, therefore is not visited
		std::cout << "pushed into queue Left: " << point.x << ", " << point.y - dot_distance << ", " << point.z << std::endl;	
		q.push(pointLeft); //add it to the queue
		visited_map.insert({tempLeft, true}); //add it to visited points
	}  else {
		//std::cout << "FOUND INSIDE OF MAP" << std::endl;
	}

	
	std::get<0>(tempRight) = point.x;
	std::get<1>(tempRight) = point.y + dot_distance;
	std::get<2>(tempRight) = point.z;
	pointRight.x = point.x;
	pointRight.y  = point.y + dot_distance;
	pointRight.z  = point.z;
	search = visited_map.find(tempRight);
	if (search == visited_map.end() && isPointInside(scan, pointRight)) { //did not manage to find it in the map, therefore is not visited
		std::cout << "pushed into queue Right: " << point.x << ", " << point.y + dot_distance << ", " << point.z << std::endl;	
		q.push(pointRight); //add it to the queue
		visited_map.insert({tempRight, true}); //add it to visited points
	}  else {
		//std::cout << "FOUND INSIDE OF MAP" << std::endl;
	}
	
	std::get<0>(tempForward) = point.x + dot_distance;
	std::get<1>(tempForward) = point.y;
	std::get<2>(tempForward) = point.z;
	pointForward.x = point.x + dot_distance;
	pointForward.y  = point.y;
	pointForward.z  = point.z;
	search = visited_map.find(tempForward);
	if (search == visited_map.end() && isPointInside(scan, pointForward)) { //did not manage to find it in the map, therefore is not visited
		std::cout << "pushed into queue Forward: " << point.x + dot_distance << ", " << point.y << ", " << point.z << std::endl;	
		q.push(pointForward); //add it to the queue
		visited_map.insert({tempForward, true}); //add it to visited points
	}  else {
		//std::cout << "FOUND INSIDE OF MAP" << std::endl;
	}
	
	std::get<0>(tempBackward) = point.x - dot_distance;
	std::get<1>(tempBackward) = point.y;
	std::get<2>(tempBackward) = point.z;
	pointBackward.x = point.x - dot_distance;
	pointBackward.y  = point.y;
	pointBackward.z  = point.z;
	search = visited_map.find(tempBackward);
	if (search == visited_map.end() && isPointInside(scan, pointBackward)) { //did not manage to find it in the map, therefore is not visited
		std::cout << "pushed into queue Back: " << point.x - dot_distance << ", " << point.y << ", " << point.z << std::endl;	
		q.push(pointBackward); //add it to the queue
		visited_map.insert({tempBackward, true}); //add it to visited points
	} 	 else {
		//std::cout << "FOUND INSIDE OF MAP" << std::endl;
	}

}

void cloud_cb(const sensor_msgs::PointCloud2& cloud_msg) {
	std::cout << "start" << std::endl;
	pcl::PCLPointCloud2 pcl_cloud_msg_2;
	pcl_conversions::toPCL(cloud_msg, pcl_cloud_msg_2);
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_msg;
	pcl::fromPCLPointCloud2( pcl_cloud_msg_2, pcl_cloud_msg);
	
	//create the LidarScan from PointCloud
	LidarScan scan;
    decomposeLidarScanIntoPlanes(pcl_cloud_msg, scan);
	
	pcl::PointXYZ temp;
	pcl::PointXYZ start = pcl::PointXYZ(0,0,0);
	std::unordered_map<std::tuple<float, float, float>, bool, key_hash, key_equal> visited_map;
	//std::unordered_map<std::tuple, bool> visited_map;
	
	pcl::PointCloud<pcl::PointXYZ> out_cloud;
	std::queue<pcl::PointXYZ> q;
	
	q.push(start);
	
	while (!q.empty()) {
		//std::cout << "Queue size 0: " << q.size() << std::endl;
		
		temp = q.front();
		q.pop();
		//std::cout << "Queue size 1: " << q.size() << std::endl;
	
		out_cloud.push_back (temp); //add it to the cloud to be publish
		add_to_queue(visited_map, temp, q, scan); //add points around temp to the queue
		
		//std::cout << "Queue size 2:  " << q.size() << std::endl;
	}
	
	
//	std::cout << scan.size() << std::endl;
	out_cloud.header.frame_id = "X1/base_link";
	out_cloud.header.stamp = pcl_conversions::toPCL(cloud_msg.header.stamp);
	pub.publish(out_cloud);
	std::cout << "OVER" << std::endl;
	//ros::shutdown();
}

int main (int argc, char* argv[]) {
	ros::init (argc, argv, "dots_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe ("/X1/points/", 1, cloud_cb);
	//pub = nh.advertise<sensor_msgs::PointCloud> ("output", 1);
	pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("output", 1);	
	ros::spin();
}
