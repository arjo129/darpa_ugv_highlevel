

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
const int fill_gap_amt = 10;
static LidarScan scan;
static pcl::PCLPointCloud2 pcl_cloud_msg_2;
static pcl::PointCloud<pcl::PointXYZ> pcl_cloud_msg;
static pcl::PointCloud<pcl::PointXYZ> original_cloud_msg;
static pcl::PointXYZ origin;
static bool origin_is_updated;
static int index_global = -1;

static int current_frame = 0;
const int every_x_frames = 2;
const bool disable_skipping = true;


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



int add_to_queue(std::unordered_map<pcl::PointXYZ,int,key_hash,key_equal>& visited_map, pcl::PointXYZ& point,std::queue<std::tuple<pcl::PointXYZ, bool>>& q, LidarScan& scan, graph_msgs::Edges& e, int& index_global, graph_msgs::GeometryGraph& out_cloud_2, geometry_msgs::Point& out_cloud_2_point, std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>& visited_map_2, int& index_local, tf2_ros::Buffer& tfBuffer, ros::Time& timeStamp) 
{
	pcl::PointXYZ pointUp, pointDown, pointLeft, pointRight, pointForward, pointBackward;
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>::iterator search;
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>::iterator search_global;
	std::cout << "index_global (add_to_queue()): " << index_global << std::endl;

	geometry_msgs::PointStamped geo_point, transformed_geo_pt; 
	geo_point.header.frame_id = "X1/world";
	geo_point.header.stamp = timeStamp;
	transformed_geo_pt.header.frame_id = "X1/base_link/front_laser";
	geo_point.point.x = point.x;
	geo_point.point.y = point.y;
	geo_point.point.z = point.z;
	tfBuffer.transform(geo_point, transformed_geo_pt, "X1/base_link/front_laser");

	std::cout << "index_global (add_to_queue()): " << index_global << std::endl;

	pcl::PointXYZ transformed_pt;
	
	pointUp.x = point.x;
	pointUp.y = point.y;
	pointUp.z = point.z + dot_distance;	
	std::cout << "after setting pointUp" << std::endl;
	if (pointUp.z < max_z) {
		transformed_pt.x = transformed_geo_pt.point.x;
		transformed_pt.y = transformed_geo_pt.point.y;
		transformed_pt.z = transformed_geo_pt.point.z  + dot_distance;
		if (isPointInside(scan, transformed_pt)) { //if neighbouring point is still inside of scan, we just add it to the list of points to visit
			search = visited_map_2.find(pointUp);
			search_global = visited_map.find(pointUp);
			if (search == visited_map_2.end()) { //if locally, have not visited the point yet
				++index_local;
				visited_map_2.insert({pointUp, index_local}); //add it to locally visited points
				if (search == visited_map.end()) {//if we have not visited this point globally yet, we add it to geometry message
					std::cout << "GLOBALLY - TRUE: pushed into queue Up: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
					q.push(std::make_tuple(pointUp, true)); //add it to the queue
					++index_global;
					visited_map.insert({pointUp, index_global}); //add it to globally visited points
				} else { //have visited this neighbouring point before
					std::cout << "GLOBALLY - FALSE Up:  have visited this neighbouring point before: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
					q.push(std::make_tuple(pointUp, false)); //add it to the queue
				}
			} else {
				std::cout << "LOCALLY Up: have visited this neighbouring point before: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointUp
			} else {
				e.node_ids.push_back(visited_map[pointUp]);
			}

		} else {
			std::cout << "OUTSIDE of laserScan Up: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
		}
	} else {
		std::cout << "exceed max_z: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << " || max_z: " << max_z << std::endl;	
	}

	pointDown.x = point.x;
	pointDown.y = point.y;
	pointDown.z = point.z - dot_distance;
	if (pointDown.z < max_z) {
		transformed_pt.x = transformed_geo_pt.point.x;
		transformed_pt.y = transformed_geo_pt.point.y;
		transformed_pt.z = transformed_geo_pt.point.z  - dot_distance;
		if (isPointInside(scan, transformed_pt)) { //if neighbouring point is still inside of scan 
			search = visited_map_2.find(pointDown);
			search_global = visited_map.find(pointDown);
			if ( search == visited_map_2.end() ) { //if locally, I have not visited the point yet
				++index_local;
				visited_map_2.insert({pointDown, index_local}); //add it to locally visited points
				if (search == visited_map.end()) {
					std::cout << "GLOBALLY - TRUE: pushed into queue Down: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z  << std::endl;	
					q.push(std::make_tuple(pointDown, true));
					++index_global;
					visited_map.insert({pointDown, index_global}); //add it to globally visited points
				} else { //globally, I have visited this point before, so add the index_global of pointDown to the the edge list of the original point
					std::cout << "GLOBALLY - FALSE Down: have visited this neighbouring point before: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
					q.push(std::make_tuple(pointDown, false));
				}
			} else {
				std::cout << "LOCALLY Down: have visited this neighbouring point before: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointDown
			} else {
				e.node_ids.push_back(visited_map[pointDown]);
			}

		} else {
			std::cout << "OUTSIDE of laserScan Down: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
		}
	}
	
	pointLeft.x = point.x;
	pointLeft.y = point.y - dot_distance;
	pointLeft.z = point.z;	
	if (pointLeft.z < max_z) {		
		transformed_pt.x = transformed_geo_pt.point.x;
		transformed_pt.y = transformed_geo_pt.point.y - dot_distance;
		transformed_pt.z = transformed_geo_pt.point.z;
		if (isPointInside(scan, transformed_pt)) { 
			search = visited_map_2.find(pointLeft);
			search_global = visited_map.find(pointLeft);
			if (search == visited_map_2.end()) {
				++index_local;	
				visited_map_2.insert({pointLeft, index_local}); //add it to locally visited points
				if (search == visited_map.end()) {
					std::cout << "GLOBALLY - TRUE: pushed into queue Left: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;
					q.push(std::make_tuple(pointLeft, true));
					++index_global;
					visited_map.insert({pointLeft, index_global}); //add it to visited points
				} else { //if i have visited pointLeft before
					std::cout << "GLOBALLY - FALSE Left: have visited this neighbouring point before: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
					q.push(std::make_tuple(pointLeft, false));
				}
			} else {
				std::cout << "OUTSIDE of laserScan Left: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointLeft
			} else {
				e.node_ids.push_back(visited_map[pointLeft]);
			}

		} else {
			std::cout << "LOCALLY Left: have visited this neighbouring point before: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
		}
	}

	
	pointRight.x = point.x;
	pointRight.y  = point.y + dot_distance;
	pointRight.z  = point.z;
	if (pointRight.z < max_z) {		
		transformed_pt.x = transformed_geo_pt.point.x;
		transformed_pt.y = transformed_geo_pt.point.y + dot_distance;
		transformed_pt.z = transformed_geo_pt.point.z;
		if (isPointInside(scan, transformed_pt)) { 
			search = visited_map_2.find(pointRight);
			search_global = visited_map.find(pointRight);
			if (search == visited_map_2.end()) {
				std::cout << "GLOBALLY - TRUE : pushed into queue Right: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;
				++index_local;	
				visited_map_2.insert({pointRight, index_local}); //add it to locally visited points
				if (search == visited_map.end()) {
					q.push(std::make_tuple(pointRight, true));
					++index_global;
					visited_map.insert({pointRight, index_global}); //add it to visited points
				} else { //if i have visited pointRight before
					std::cout << "GLOBALLY - FALSE Right: have visited this neighbouring point before: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
					q.push(std::make_tuple(pointRight, false));
				}
			} else {
				std::cout << "LOCALLY Right: have visited this neighbouring point before: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
			} 

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointRight
			} else {
				e.node_ids.push_back(visited_map[pointRight]);
			}

		} else {
			std::cout << "OUTSIDE of laserScan Right: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
		}
	}
	
	
	pointForward.x = point.x + dot_distance;
	pointForward.y  = point.y;
	pointForward.z  = point.z;
	if (pointForward.z < max_z) {		
		transformed_pt.x = transformed_geo_pt.point.x + dot_distance;
		transformed_pt.y = transformed_geo_pt.point.y;
		transformed_pt.z = transformed_geo_pt.point.z;
		if (isPointInside(scan, transformed_pt)) { 
			search = visited_map_2.find(pointForward);
			search_global = visited_map.find(pointForward);
			if (search == visited_map_2.end()) {
				//std::cout << "pushed into queue Left: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;
				++index_local;	
				visited_map_2.insert({pointForward, index_local}); //add it to locally visited points
				if (search == visited_map.end()) {
					std::cout << "GLOBALLY - TRUE pushed into queue Forward: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
					q.push(std::make_tuple(pointForward, true));
					++index_global;
					visited_map.insert({pointForward, index_global}); //add it to visited points
				} else { //if i have visited pointForward before
					std::cout << "GLOBALLY - FALSE Forward: have visited this neighbouring point before: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
					q.push(std::make_tuple(pointForward, false));
				}
			} else {
				std::cout << "LOCALLY Forward: have visited this neighbouring point before: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointForward
			} else {
				e.node_ids.push_back(visited_map[pointForward]);
			}

		} else {
			std::cout << "OUTSIDE of laserScan Forward: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
		}
	}
	

	pointBackward.x = point.x - dot_distance;
	pointBackward.y  = point.y;
	pointBackward.z  = point.z;
	if (pointBackward.z < max_z) {
		transformed_pt.x = transformed_geo_pt.point.x - dot_distance;
		transformed_pt.y = transformed_geo_pt.point.y;
		transformed_pt.z = transformed_geo_pt.point.z;
		if (isPointInside(scan, transformed_pt)) { //if pointBackward is still inside of scan, we add it to the list of points to visit
			search = visited_map_2.find(pointBackward);
			search_global = visited_map.find(pointBackward);
			if (search == visited_map_2.end()) { //if locally, have not visited pointBackward yet
				++index_local;
				visited_map_2.insert({pointBackward, index_local}); //add it to locally visited points
				if (search == visited_map.end()) {//if we have not visited this point globally yet, we add it to geometry message
					std::cout << "GLOBALLY - TRUE pushed into queue Backward: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
					q.push(std::make_tuple(pointBackward, true));
					++index_global;
					visited_map.insert({pointBackward, index_global}); //add it to globally visited points
				} else { //have visited this neighbouring point before
					std::cout << "GLOBALLY - FALSE Backward: have visited this neighbouring point before: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
					q.push(std::make_tuple(pointBackward, false));
				}
			} else {
					std::cout << "LOCALLY Backward: have visited this neighbouring point before: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointBackward
			} else {
				e.node_ids.push_back(visited_map[pointBackward]);
			}

		} else {
			std::cout << "OUTSIDE of laserScan Backward: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;
		}
	}
	
	std::cout << "Neighbouring Nodes:" << std::endl;
	for (int i : e.node_ids) {
		std::cout << i << std::endl;
	}
	//std::cout << "index_global 2: " << index_global << std::endl;
}

void cloud_cb(pcl::PointXYZ start, pcl::PointCloud<pcl::PointXYZ>& original_cloud_msg, tf2_ros::Buffer& tfBuffer, ros::Time& timeStamp) {

	if ((original_cloud_msg.size() < 1) || !origin_is_updated) { //|| (start.x == 0 && start.y == 0 && start.z == 0)) {
		std::cout << "Returning at start: !origin_is_updated: " << !origin_is_updated << " || original_cloud_msg.size(): " << original_cloud_msg.size() << "\n";
		return;
	}
	clock_t tStart = clock();
	
	//create the LidarScan from PointCloud
    decomposeLidarScanIntoPlanes(original_cloud_msg, scan);
	for (int i = 0; i < scan.size(); i++) {
		fillGaps(scan[i].scan, fill_gap_amt);
	}
	
	std::tuple<pcl::PointXYZ, bool> temp_tuple; //declare a tuple to keep the tuple I'll dequeue later
	pcl::PointXYZ temp; //declare a point which I'll get from the de-queued tuple
	static std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal> visited_map; //to keep throughout the run - the one keeping all the global points
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal> visited_map_2; //per call back - to keep track of my locally visited BFS points, etc
	
	pcl::PointCloud<pcl::PointXYZ> out_cloud;
	std::queue<std::tuple<pcl::PointXYZ, bool>> q; //declare the queue of tuples - if true then add to geo msg. true/false is set in the above method
	
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
		std::cout << "index_global cloud_cb(): " << index_global << std::endl;
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
		add_to_queue(visited_map, temp, q, scan, e, index_global, out_cloud_2, out_cloud_2_point, visited_map_2, index_local, tfBuffer, timeStamp); //add points around temp to the queue
		// std::cout << "to_add: " << to_add << std::endl;
		if (to_add) {
			out_cloud_2.edges.push_back(e);	
		}
		if (index_local > 100000) {
			std::cout << "INDEX_LOCAL EXCEEDED" << "\n";
			break;
		}
	}
	
	std::vector<uint8_t> arr(out_cloud_2.nodes.size());
	// std::cout<< out_cloud_2.nodes.size() << "\n"; //number of points
	out_cloud_2.explored = arr;
	
	out_cloud_2.header.frame_id = "X1/world";
	out_cloud_2.header.stamp = pcl_conversions::fromPCL(original_cloud_msg.header.stamp);
	//pub.publish(out_cloud);
	pub2.publish(out_cloud_2);
	//std::cout << "OVER" << std::endl;
	printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
	// ros::shutdown();
}

void transform_the_cloud(const sensor_msgs::PointCloud2& cloud_msg) { //called when there is a new lidar scan in the form of a point cloud being sent in 
	if (disable_skipping || current_frame >= every_x_frames ) {
		current_frame = 0;
	} else {
		current_frame += 1;
		std::cout << "returning due SKIP" << std::endl;
		return;
	}

	//get the position of the robot
	pcl::PointXYZ start;
	start.x = origin.x; //where origin is the robot's location updated by integrated_to_init in the update_origin() callback method
	start.y = origin.y;
	start.z = origin.z;
	std::cout << "robot's location: " << start.x << ", " << start.y << ", " << start.z << std::endl;	
	ros::Time timeStamp = cloud_msg.header.stamp;

	//transform the pointcloud: X1/points
	static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	sensor_msgs::PointCloud2 cloud_transformed;
	try {
		if (tfBuffer.canTransform("X1/world", "X1/base_link/front_laser", cloud_msg.header.stamp, ros::Duration(3.0))) {
			transformStamped = tfBuffer.lookupTransform("X1/world", "X1/base_link/front_laser", cloud_msg.header.stamp);
			tf2::doTransform (cloud_msg, cloud_transformed, transformStamped); //tf2::doTransform(cloud_in, cloud_out, transform)

			//publish the transformed cloud
			pub3.publish(cloud_transformed); 

			//convert the original cloud for use in cloud_cb
			pcl_conversions::toPCL(cloud_msg, pcl_cloud_msg_2);
			pcl::fromPCLPointCloud2( pcl_cloud_msg_2, original_cloud_msg); 

			//call the grapher method
			cloud_cb(start, original_cloud_msg, tfBuffer, timeStamp);
		}

	}  catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
		return;
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
