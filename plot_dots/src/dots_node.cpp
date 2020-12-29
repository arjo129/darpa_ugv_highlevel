/**
I have 3 methods and 2 structs here. 
The 2 structs are for the hashMap I use to track visited dots. 
The 3 methods are main, cloud_cb and add_to_queue. 
I make use of a BFS style to structure how I go about adding the dots. The initial dot is 0,0,0 which is added to the queue. When it dequeues that dot, that dot is added to out_cloud which publishes it at the end
I will also call add_to_queue which looks at the possible dots around it in 3 dimensions (exclude diagonals). If I have not visited those dots and they still loe within the lidar scans, I will add them to the queue.
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
//#include <tuple>
#include <queue> 

#include <boost/functional/hash.hpp>
#include <math.h> 
#include <time.h>

#include <graph_msgs/Edges.h>
#include <graph_msgs/GeometryGraph.h>
#include "geometry_msgs/Point.h"

ros::Publisher pub;
ros::Publisher pub2;
const float dot_distance = 1.5;
const float max_z = 15;
const float EPSILON = 0.0001;

struct key_hash {
  std::size_t operator()(const pcl::PointXYZ& k) const {
      using boost::hash_value;
      using boost::hash_combine;

      // Start with a hash value of 0    .
      std::size_t seed = 0;

	  //hash_combine(seed,hash_value(std::get<0>(k)));
	  //hash_combine(seed,hash_value(std::get<1>(k)));
	  //hash_combine(seed,hash_value(std::get<2>(k)));
	  hash_combine(seed,hash_value(k.x));
	  hash_combine(seed,hash_value(k.y));
	  hash_combine(seed,hash_value(k.z));

      // Return the result.
      return seed;
  }
};

struct key_equal : public std::binary_function<pcl::PointXYZ, pcl::PointXYZ, bool> {
  bool operator()(const pcl::PointXYZ& v0, const pcl::PointXYZ& v1) const {
   return (
   fabs(v0.x - v1.x) < EPSILON &&
   fabs(v0.y - v1.y) < EPSILON &&
   fabs(v0.z - v1.z) < EPSILON
  );
  }
};



int add_to_queue(std::unordered_map<pcl::PointXYZ,int,key_hash,key_equal>& visited_map, pcl::PointXYZ& point,std::queue<pcl::PointXYZ>& q, LidarScan& scan, graph_msgs::Edges& e, int& index) 
{
	//std::tuple<float, float, float> tempUp, tempDown, tempLeft, tempRight, tempForward, tempBackward;
	pcl::PointXYZ pointUp, pointDown, pointLeft, pointRight, pointForward, pointBackward;
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>::iterator search;
	//std::cout << "Index 1: " << index << std::endl;
	
	pointUp.x = point.x;
	pointUp.y  = point.y;
	pointUp.z  = point.z + dot_distance;	
	if (pointUp.z < max_z) {
		search = visited_map.find(pointUp);
		if (isPointInside(scan, pointUp)) { //if neighbouring point is still inside of scan
			if (search == visited_map.end()) { //if have not visited the point yet, add it to the queue to visit
				//std::cout << "pushed into queue Up: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
				++index;
				e.node_ids.push_back(index); //index should be referring to pointUp
				q.push(pointUp); //add it to the queue
				visited_map.insert({pointUp, index}); //add it to visited points
			} else { //have visited this neighbouring point before
				//std::cout << "have visited this neighbouring point before: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
				e.node_ids.push_back(visited_map[pointUp]);
			}
		}
	}

	pointDown.x = point.x;
	pointDown.y  = point.y;
	pointDown.z  = point.z - dot_distance;
	if (pointDown.z < max_z) {
		search = visited_map.find(pointDown);
		if ( isPointInside(scan, pointDown)) { //if neighbouring point is still inside of scan
			if (search == visited_map.end() ) { //if have not visited the point yet, add it to the queue to visit
				//std::cout << "pushed into queue Down: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z  << std::endl;	
				++index;
				e.node_ids.push_back(index);
				q.push(pointDown); //add it to the queue
				visited_map.insert({pointDown, index}); //add it to visited points
			}  else {
				//std::cout << "have visited this neighbouring point before: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
				e.node_ids.push_back(visited_map[pointDown]);
			}
		}
	}

	
	pointLeft.x = point.x;
	pointLeft.y  = point.y - dot_distance;
	pointLeft.z  = point.z;	
	if (pointLeft.z < max_z) {		
		search = visited_map.find(pointLeft);
		if (isPointInside(scan, pointLeft)) {
			if (search == visited_map.end()) { 
				//std::cout << "pushed into queue Left: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
				++index;
				e.node_ids.push_back(index);
				q.push(pointLeft); //add it to the queue
				visited_map.insert({pointLeft, index}); //add it to visited points
			} else {
				//std::cout << "have visited this neighbouring point before: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
				e.node_ids.push_back(visited_map[pointLeft]);
			}
		}
	}

	
	pointRight.x = point.x;
	pointRight.y  = point.y + dot_distance;
	pointRight.z  = point.z;
	if (pointRight.z < max_z) {		
		search = visited_map.find(pointRight);
		if (isPointInside(scan, pointRight)) {
			if (search == visited_map.end()) { 
				//std::cout << "pushed into queue Right: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
				++index;
				e.node_ids.push_back(index);
				q.push(pointRight); //add it to the queue
				visited_map.insert({pointRight, index}); //add it to visited points
			}  else {
				//std::cout << "have visited this neighbouring point before: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
				e.node_ids.push_back(visited_map[pointRight]);
			}
		}
	}
	
	
	pointForward.x = point.x + dot_distance;
	pointForward.y  = point.y;
	pointForward.z  = point.z;
	if (pointForward.z < max_z) {			
		search = visited_map.find(pointForward);
		if (isPointInside(scan, pointForward)) {
			if (search == visited_map.end()) { 
				//std::cout << "pushed into queue Forward: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
				++index;
				e.node_ids.push_back(index);
				q.push(pointForward);
				visited_map.insert({pointForward, index});
			} else {
				//std::cout << "have visited this neighbouring point before: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
				e.node_ids.push_back(visited_map[pointForward]);
			}
		}
	}
	

	pointBackward.x = point.x - dot_distance;
	pointBackward.y  = point.y;
	pointBackward.z  = point.z;
	if (pointBackward.z < max_z) {
		search = visited_map.find(pointBackward);
		if (isPointInside(scan, pointBackward)) {
			if (search == visited_map.end()) {
				//std::cout << "pushed into queue Back: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
				++index;
				e.node_ids.push_back(index);
				q.push(pointBackward); //add it to the queue
				visited_map.insert({pointBackward, index}); //add it to visited points
			} 	 else {
				//std::cout << "have visited this neighbouring point before: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
				e.node_ids.push_back(visited_map[pointBackward]);
			}
		}
	}
	
	//std::cout << "Index 2: " << index << std::endl;
}

void cloud_cb(const sensor_msgs::PointCloud2& cloud_msg) {
	clock_t tStart = clock();
	//std::cout << "start" << std::endl;
	pcl::PCLPointCloud2 pcl_cloud_msg_2;
	pcl_conversions::toPCL(cloud_msg, pcl_cloud_msg_2);
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_msg;
	pcl::fromPCLPointCloud2( pcl_cloud_msg_2, pcl_cloud_msg);
	
	//create the LidarScan from PointCloud
	LidarScan scan;
    decomposeLidarScanIntoPlanes(pcl_cloud_msg, scan);
	
	pcl::PointXYZ temp;
	pcl::PointXYZ start = pcl::PointXYZ(0,0,0);
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal> visited_map;
	
	pcl::PointCloud<pcl::PointXYZ> out_cloud;
	std::queue<pcl::PointXYZ> q;
	
	graph_msgs::GeometryGraph out_cloud_2;
	static geometry_msgs::Point out_cloud_2_point;
	int index = -1;
	
	q.push(start);
	++index;
	pcl::PointXYZ start_point;
	start_point.x = 0;
	start_point.y = 0;
	start_point.z = 0;
	visited_map.insert({start_point, index}); //add it to visited points
	
	while (!q.empty()) {
		//std::cout << "Index 0: " << index << std::endl;
		//std::cout << "Queue size 0: " << q.size() << std::endl;
		
		temp = q.front();
		q.pop();
		//std::cout << "Queue size 1: " << q.size() << std::endl;
		out_cloud.push_back (temp); //add it to the cloud to be publish
		
		out_cloud_2_point.x = temp.x;
		out_cloud_2_point.y = temp.y;
		out_cloud_2_point.z = temp.z;
		out_cloud_2.nodes.push_back(out_cloud_2_point);
		graph_msgs::Edges e;
		add_to_queue(visited_map, temp, q, scan, e, index); //add points around temp to the queue
		out_cloud_2.edges.push_back(e);		

	}
	
	std::vector<uint8_t> arr(out_cloud_2.nodes.size());
	//std::cout<< out_cloud_2.nodes.size() << "\n"; //number of points
	out_cloud_2.explored = arr;
	
	//out_cloud.header.frame_id = "X1/base_link";
	//out_cloud.header.stamp = pcl_conversions::toPCL(cloud_msg.header.stamp);
	out_cloud_2.header = cloud_msg.header;
	//pub.publish(out_cloud);
	pub2.publish(out_cloud_2);
	//std::cout << "OVER" << std::endl;
	printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
	//ros::shutdown();
}

int main (int argc, char* argv[]) {
	ros::init (argc, argv, "dots_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe ("/X1/points/", 1, cloud_cb);
	//pub = nh.advertise<sensor_msgs::PointCloud> ("output", 1);
	//pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("output", 1);	
	pub2 = nh.advertise<graph_msgs::GeometryGraph> ("output2", 1);	
	ros::spin();
}
