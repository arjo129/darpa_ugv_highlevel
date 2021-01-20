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
#include <tuple>
#include <unordered_map>
#include <queue> 

#include <boost/functional/hash.hpp>
#include <math.h> 
#include <time.h>

#include <graph_msgs/Edges.h>
#include <graph_msgs/GeometryGraph.h>
#include "geometry_msgs/Point.h"

#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
const float dot_distance = 1.5;
const float max_z = 7;
const float EPSILON = 0.0001;
static LidarScan scan;
static pcl::PCLPointCloud2 pcl_cloud_msg_2;
static pcl::PointCloud<pcl::PointXYZ> pcl_cloud_msg;
static pcl::PointCloud<pcl::PointXYZ> original_cloud_msg;
static pcl::PointXYZ origin;
static bool origin_is_updated;
static int index_global = -1;


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



int add_to_queue(std::unordered_map<pcl::PointXYZ,int,key_hash,key_equal>& visited_map, pcl::PointXYZ& point,std::queue<std::tuple<pcl::PointXYZ, bool>>& q, LidarScan& scan, graph_msgs::Edges& e, int& index_global, graph_msgs::GeometryGraph& out_cloud_2, geometry_msgs::Point& out_cloud_2_point, std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>& visited_map_2, int& index_local, geometry_msgs::TransformStamped& transformStamped_world_to_baseLink, tf2_ros::Buffer& tfBuffer) 
{
	pcl::PointXYZ pointUp, pointDown, pointLeft, pointRight, pointForward, pointBackward;
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>::iterator search;
	std::cout << "index_global 1: " << index_global << std::endl;

	geometry_msgs::PointStamped geo_point, transformed_geo_pt; 
	geo_point.header.frame_id = "X1/world";
	transformed_geo_pt.header.frame_id = "X1/base_link/front_laser";
	geo_point.point.x = point.x;
	geo_point.point.y = point.y;
	geo_point.point.z = point.z;
	tfBuffer.transform(geo_point, transformed_geo_pt, "X1/base_link/front_laser");

	pcl::PointXYZ transformed_pt;
	
	pointUp.x = point.x;
	pointUp.y = point.y;
	pointUp.z = point.z + dot_distance;	
	std::cout << "after setting pointUp" << std::endl;
	if (pointUp.z < max_z) {
		search = visited_map_2.find(pointUp);
		if (search == visited_map_2.end()) { //if locally, have not visited the point yet
			transformed_pt.x = transformed_geo_pt.point.x;
			transformed_pt.y = transformed_geo_pt.point.y;
			transformed_pt.z = transformed_geo_pt.point.z  + dot_distance;
			if (isPointInside(scan, transformed_pt)) { //if neighbouring point is still inside of scan, we just add it to the list of points to visit
				++index_local;
				visited_map_2.insert({pointUp, index_local}); //add it to locally visited points
				search = visited_map.find(pointUp);
				if (search == visited_map.end()) {//if we have not visited this point globally yet, we add it to geometry message
					std::cout << "GLOBALLY - TRUE: pushed into queue Up: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
					q.push(std::make_tuple(pointUp, true)); //add it to the queue
					++index_global;
					e.node_ids.push_back(index_global); //index_global should be referring to pointUp
					visited_map.insert({pointUp, index_global}); //add it to globally visited points
				} else { //have visited this neighbouring point before
					std::cout << "GLOBALLY - FALSE Up:  have visited this neighbouring point before: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
					q.push(std::make_tuple(pointUp, false)); //add it to the queue
					e.node_ids.push_back(visited_map[pointUp]);
				}
			}else {
				std::cout << "OUTSIDE of laserScan Up: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
			}
		} else {
			std::cout << "LOCALLY Up: have visited this neighbouring point before: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
		}
	} else {
		std::cout << "exceed max_z: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << " || max_z: " << max_z << std::endl;	
	}

	pointDown.x = point.x;
	pointDown.y = point.y;
	pointDown.z = point.z - dot_distance;
	if (pointDown.z < max_z) {
		search = visited_map_2.find(pointDown);
		if ( search == visited_map_2.end() ) { //if locally, I have not visited the point yet
			transformed_pt.x = transformed_geo_pt.point.x;
			transformed_pt.y = transformed_geo_pt.point.y;
			transformed_pt.z = transformed_geo_pt.point.z  - dot_distance;
			if (isPointInside(scan, transformed_pt)) { //if neighbouring point is still inside of scan 
				++index_local;
				visited_map_2.insert({pointDown, index_local}); //add it to locally visited points
				search = visited_map.find(pointDown);
				if (search == visited_map.end()) {
					std::cout << "GLOBALLY - TRUE: pushed into queue Down: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z  << std::endl;	
					q.push(std::make_tuple(pointDown, true));
					++index_global;
					e.node_ids.push_back(index_global); //index_global should be referring to pointDown
					visited_map.insert({pointDown, index_global}); //add it to globally visited points
				} else { //globally, I have visited this point before, so add the index_global of pointDown to the the edge list of the original point
					std::cout << "GLOBALLY - FALSE Down: have visited this neighbouring point before: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
					e.node_ids.push_back(visited_map[pointDown]);
					q.push(std::make_tuple(pointDown, false));
				}
			} else {
				std::cout << "OUTSIDE of laserScan Down: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
			}
		} else {
			std::cout << "LOCALLY Down: have visited this neighbouring point before: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
		}
	}
	
	pointLeft.x = point.x;
	pointLeft.y = point.y - dot_distance;
	pointLeft.z = point.z;	
	if (pointLeft.z < max_z) {		
		search = visited_map_2.find(pointLeft);
		if (search == visited_map_2.end()) {
			transformed_pt.x = transformed_geo_pt.point.x;
			transformed_pt.y = transformed_geo_pt.point.y - dot_distance;
			transformed_pt.z = transformed_geo_pt.point.z;
			if (isPointInside(scan, transformed_pt)) { 
				++index_local;	
				visited_map_2.insert({pointLeft, index_local}); //add it to locally visited points
				search = visited_map.find(pointLeft);
				if (search == visited_map.end()) {
					std::cout << "GLOBALLY - TRUE: pushed into queue Left: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;
					q.push(std::make_tuple(pointLeft, true));
					++index_global;
					e.node_ids.push_back(index_global);
					visited_map.insert({pointLeft, index_global}); //add it to visited points
				} else { //if i have visited pointLeft before
					std::cout << "GLOBALLY - FALSE Left: have visited this neighbouring point before: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
					e.node_ids.push_back(visited_map[pointLeft]);
					q.push(std::make_tuple(pointLeft, false));
				}
			} else {
				std::cout << "OUTSIDE of laserScan Left: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
			}
		} else {
			std::cout << "LOCALLY Left: have visited this neighbouring point before: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
		}
	}

	
	pointRight.x = point.x;
	pointRight.y  = point.y + dot_distance;
	pointRight.z  = point.z;
	if (pointRight.z < max_z) {		
		search = visited_map_2.find(pointRight);
		if (search == visited_map_2.end()) {
			transformed_pt.x = transformed_geo_pt.point.x;
			transformed_pt.y = transformed_geo_pt.point.y + dot_distance;
			transformed_pt.z = transformed_geo_pt.point.z;
			if (isPointInside(scan, transformed_pt)) { 
				std::cout << "GLOBALLY - TRUE : pushed into queue Right: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;
				++index_local;	
				visited_map_2.insert({pointRight, index_local}); //add it to locally visited points
				search = visited_map.find(pointRight);
				if (search == visited_map.end()) {
					q.push(std::make_tuple(pointRight, true));
					++index_global;
					e.node_ids.push_back(index_global);
					visited_map.insert({pointRight, index_global}); //add it to visited points
				} else { //if i have visited pointRight before
					std::cout << "GLOBALLY - FALSE Right: have visited this neighbouring point before: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
					e.node_ids.push_back(visited_map[pointRight]);
					q.push(std::make_tuple(pointRight, false));
				}
			} else {
				std::cout << "OUTSIDE of laserScan Right: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
			} 
		} else {
			std::cout << "LOCALLY Right: have visited this neighbouring point before: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
		}
	}
	
	
	pointForward.x = point.x + dot_distance;
	pointForward.y  = point.y;
	pointForward.z  = point.z;
	if (pointForward.z < max_z) {		
		search = visited_map_2.find(pointForward);
		if (search == visited_map_2.end()) {
			transformed_pt.x = transformed_geo_pt.point.x + dot_distance;
			transformed_pt.y = transformed_geo_pt.point.y;
			transformed_pt.z = transformed_geo_pt.point.z;
			if (isPointInside(scan, transformed_pt)) { 
				//std::cout << "pushed into queue Left: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;
				++index_local;	
				visited_map_2.insert({pointForward, index_local}); //add it to locally visited points
				search = visited_map.find(pointForward);
				if (search == visited_map.end()) {
					std::cout << "GLOBALLY - TRUE pushed into queue Forward: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
					q.push(std::make_tuple(pointForward, true));
					++index_global;
					e.node_ids.push_back(index_global);
					visited_map.insert({pointForward, index_global}); //add it to visited points
				} else { //if i have visited pointForward before
					std::cout << "GLOBALLY - FALSE Forward: have visited this neighbouring point before: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
					e.node_ids.push_back(visited_map[pointForward]);
					q.push(std::make_tuple(pointForward, false));
				}
			} else {
				std::cout << "OUTSIDE of laserScan Forward: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
			}
		} else {
			std::cout << "LOCALLY Forward: have visited this neighbouring point before: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
		}
	}
	

	pointBackward.x = point.x - dot_distance;
	pointBackward.y  = point.y;
	pointBackward.z  = point.z;
	if (pointBackward.z < max_z) {
		search = visited_map_2.find(pointBackward);
		if (search == visited_map_2.end()) { //if locally, have not visited pointBackward yet
			transformed_pt.x = transformed_geo_pt.point.x - dot_distance;
			transformed_pt.y = transformed_geo_pt.point.y;
			transformed_pt.z = transformed_geo_pt.point.z;
			if (isPointInside(scan, transformed_pt)) { //if pointBackward is still inside of scan, we add it to the list of points to visit
				++index_local;
				visited_map_2.insert({pointBackward, index_local}); //add it to locally visited points
				search = visited_map.find(pointBackward);
				if (search == visited_map.end()) {//if we have not visited this point globally yet, we add it to geometry message
					std::cout << "GLOBALLY - TRUE pushed into queue Backward: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
					q.push(std::make_tuple(pointBackward, true));
					++index_global;
					e.node_ids.push_back(index_global); //index_global should be referring to pointBackward
					visited_map.insert({pointBackward, index_global}); //add it to globally visited points
				} else { //have visited this neighbouring point before
					std::cout << "GLOBALLY - FALSE Backward: have visited this neighbouring point before: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
					e.node_ids.push_back(visited_map[pointBackward]);
					q.push(std::make_tuple(pointBackward, false));
				}
			} else {
				std::cout << "OUTSIDE of laserScan Backward: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
			}
		} else {
			std::cout << "LOCALLY Backward: have visited this neighbouring point before: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
		}
	}
	
	//std::cout << "index_global 2: " << index_global << std::endl;
}

void cloud_cb(const pcl::PointCloud<pcl::PointXYZ>& cloud_msg, 	pcl::PointXYZ start, geometry_msgs::TransformStamped& transformStamped_world_to_baseLink, pcl::PointCloud<pcl::PointXYZ>& original_cloud_msg, tf2_ros::Buffer& tfBuffer) {

	if ((cloud_msg.size() < 1) || !origin_is_updated) { //|| (start.x == 0 && start.y == 0 && start.z == 0)) {
		std::cout << "Returning at start: !origin_is_updated: " << !origin_is_updated << " || cloud_msg.size(): " << cloud_msg.size() << "\n";
		return;
	}
	clock_t tStart = clock();
	
	//create the LidarScan from PointCloud
    decomposeLidarScanIntoPlanes(original_cloud_msg, scan);
	
	std::tuple<pcl::PointXYZ, bool> temp_tuple;
	pcl::PointXYZ temp;
	static std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal> visited_map; //to keep throughout the run
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal> visited_map_2; //per call back
	
	pcl::PointCloud<pcl::PointXYZ> out_cloud;
	std::queue<std::tuple<pcl::PointXYZ, bool>> q; //use a tuple here, if true then add to geo msg. true/false is set in the above method
	
	static graph_msgs::GeometryGraph out_cloud_2;
	static geometry_msgs::Point out_cloud_2_point;
	int index_local = -1;
	
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>::iterator search = visited_map.find(start);
	if (search == visited_map.end()) { //check if start is inside visited_map. if yes, add it visited_map_2 only and is false. if no, add it to both and is true
		++index_local;
		++index_global;
		visited_map.insert({start, index_global}); //add it to visited points
		visited_map_2.insert({start, index_local}); //add it to visited points
		q.push(std::make_tuple(start, true)); 
	} else {	
		++index_local;
		visited_map_2.insert({start, index_local}); //add it to visited points
		q.push(std::make_tuple(start, false)); 
	}
	
	while (!q.empty()) {
		std::cout << "index_global 0: " << index_global << std::endl;
		//std::cout << "Queue size 0: " << q.size() << std::endl;
		
		temp_tuple = q.front();
		q.pop();
		temp = std::get<0>(temp_tuple);
		bool to_add = std::get<1>(temp_tuple);
		// std::cout << "to_add: " << to_add << std::endl;

		if (to_add) { //check if true or false from the tuple
			out_cloud_2_point.x = temp.x;
			out_cloud_2_point.y = temp.y;
			out_cloud_2_point.z = temp.z;
			out_cloud_2.nodes.push_back(out_cloud_2_point);
		}
		
		graph_msgs::Edges e;
		add_to_queue(visited_map, temp, q, scan, e, index_global, out_cloud_2, out_cloud_2_point, visited_map_2, index_local, transformStamped_world_to_baseLink, tfBuffer); //add points around temp to the queue
		std::cout << "to_add: " << to_add << std::endl;
		if (to_add) {
			out_cloud_2.edges.push_back(e);	
		}
		// if (index_local > 100000) {
		// 	std::cout << "INDEX_LOCAL EXCEEDED" << "\n";
		// 	break;
		// }
	}
	
	std::vector<uint8_t> arr(out_cloud_2.nodes.size());
	// std::cout<< out_cloud_2.nodes.size() << "\n"; //number of points
	out_cloud_2.explored = arr;
	
	out_cloud_2.header.frame_id = "X1/world";
	out_cloud_2.header.stamp = pcl_conversions::fromPCL(cloud_msg.header.stamp);
	//pub.publish(out_cloud);
	pub2.publish(out_cloud_2);
	//std::cout << "OVER" << std::endl;
	printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
	// ros::shutdown();
}

void transform_the_cloud(const sensor_msgs::PointCloud2& cloud_msg) {
	//get the position of the robot
	pcl::PointXYZ start;
	start.x = origin.x;
	start.y = origin.y;
	start.z = origin.z;
	std::cout << "start: " << start.x << ", " << start.y << ", " << start.z << std::endl;	

	//transform the pointcloud: X1/points
	static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::TransformStamped transformStamped_world_to_baseLink;
	sensor_msgs::PointCloud2 cloud_transformed;
	try {
		transformStamped = tfBuffer.lookupTransform("X1/world", "X1/base_link/front_laser", ros::Time(0));
		transformStamped_world_to_baseLink = tfBuffer.lookupTransform("X1/base_link/front_laser", "X1/world", ros::Time(0));
		tf2::doTransform (cloud_msg, cloud_transformed, transformStamped); //(cloud_in, cloud_out, transform)

		//publish the transformed cloud
		pub3.publish(cloud_transformed); 

		//convert the transformed cloud for use in cloud_cb
		pcl_conversions::toPCL(cloud_transformed, pcl_cloud_msg_2);
		pcl::fromPCLPointCloud2( pcl_cloud_msg_2, pcl_cloud_msg); 

		//convert the original cloud for use in cloud_cb
		pcl_conversions::toPCL(cloud_msg, pcl_cloud_msg_2);
		pcl::fromPCLPointCloud2( pcl_cloud_msg_2, original_cloud_msg); 

		//call the grapher method
		cloud_cb(pcl_cloud_msg, start, transformStamped_world_to_baseLink, original_cloud_msg, tfBuffer);

	}  catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
    }
}

void update_origin(const nav_msgs::Odometry nav_odo) {
	float origin_arr[3];
	origin_arr[0] = nav_odo.pose.pose.position.x;
	origin_arr[1] = nav_odo.pose.pose.position.y;
	origin_arr[2] = nav_odo.pose.pose.position.z;
	for (int i = 0; i < 3; i++) {
		float t = origin_arr[i];
		if (t == 0) {
			continue;
		}
		float remainder = fmod(t, dot_distance);
		origin_arr[i] = t - remainder;
	}
	origin.x = origin_arr[0];
	origin.y = origin_arr[1];
	origin.z = origin_arr[2];
	origin_is_updated = true;
}

int main (int argc, char* argv[]) {
	ros::init (argc, argv, "dots_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe ("/X1/points/", 1, transform_the_cloud);
	ros::Subscriber sub3 = nh.subscribe ("integrated_to_init", 1, update_origin);
	pub2 = nh.advertise<graph_msgs::GeometryGraph> ("output2", 1);	
	pub3 = nh.advertise<sensor_msgs::PointCloud2> ("transformed_point_cloud", 1);	
	

	ros::spin();
}
