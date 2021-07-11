/*
You can just copy and paste this into the src folder of the plot_dots folder. 

The first 107 lines are imports and declaration of variables. 

Total of 4 impt methods: 
(1) main -                line 544
(2) transform_the_cloud - line 509
(3) cloud_cb -            line 406
(4) add_to_queue -        line 150

Then there is 1 helper method: 
(1) convert_local_to_global - line 111

And finally, 1 method to check the angular speed:
(1) to_skip_or_not_to_skip - line 533

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>

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
#include <string>

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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>


#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
const float dot_distance = 1.5; //1.5 or 3
const float max_z = 7;
const float EPSILON = 0.0001;
const int fill_gap_amt = 10;
pcl::PCLPointCloud2 pcl_cloud_msg_2;
static pcl::PointXYZ origin;
static bool origin_is_updated;
static int index_global = -1;
bool to_skip;
std::string world_frame;

tf2_ros::Buffer tfBuffer;

static int current_frame = 0;
const int every_x_frames = 2;
const bool disable_skipping = true;

struct key_hash {
  std::size_t operator()(const pcl::PointXYZ& k) const {
      using boost::hash_value;
      using boost::hash_combine;

      // Start with a hash value of 0    .
      std::size_t seed = 0;

	  hash_combine(seed,hash_value(round(k.x)));
	  hash_combine(seed,hash_value(round(k.y)));
	  hash_combine(seed,hash_value(round(k.z)));

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

static std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal> visited_map; //to keep throughout the run - the one keeping all the global points
static graph_msgs::GeometryGraph out_cloud_2; // for visualisation - the overall structure



bool convert_local_to_global(pcl::PointXYZ& global_point, pcl::PointXYZ& local_point, ros::Time& timeStamp, geometry_msgs::TransformStamped& transformStamped_local_to_global) { 
	geometry_msgs::PoseStamped global_point_stamped, local_point_stamped;
	local_point_stamped.pose.position.x = local_point.x;
	local_point_stamped.pose.position.y = local_point.y;
	local_point_stamped.pose.position.z = local_point.z;
	local_point_stamped.pose.orientation.x = 0.0;
	local_point_stamped.pose.orientation.y = 0.0;
	local_point_stamped.pose.orientation.z = 0.0;
	local_point_stamped.pose.orientation.w = 1.0;
	local_point_stamped.header.stamp = timeStamp;
	local_point_stamped.header.frame_id = "X1/base_link/front_laser";

	try {
        tf2::doTransform(local_point_stamped, global_point_stamped, transformStamped_local_to_global); //tf2::doTransform(point_in, point_out, transform)
	} catch (tf2::TransformException &ex) {
			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
			return false;
	}
	
	float origin_arr[3];
	origin_arr[0] = global_point_stamped.pose.position.x;
	origin_arr[1] = global_point_stamped.pose.position.y;
	origin_arr[2] = global_point_stamped.pose.position.z;
	for (int i = 0; i < 3; i++) {
		float t = origin_arr[i];
		if (t == 0) {
			continue;
		}
		float remainder = fmod(t, dot_distance);
		origin_arr[i] = t - remainder;
	}
	global_point.x = origin_arr[0];
	global_point.y = origin_arr[1];
	global_point.z = origin_arr[2];
	return true;
}


void add_to_queue(  geometry_msgs::TransformStamped& transformStamped_local_to_global,
					pcl::PointXYZ& local_point,
					std::queue<std::tuple<pcl::PointXYZ, pcl::PointXYZ, bool>>& q, 
					LidarScan& scan,
					graph_msgs::Edges& e, 
					int& index_global, 
					std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>& visited_map_2, 
					int& index_local, 
					ros::Time& timeStamp,
					bool debug_bool) 
{
	pcl::PointXYZ pointUp, pointDown, pointLeft, pointRight, pointForward, pointBackward; //the neighbouring points in the GLOBAL frame
	pcl::PointXYZ transformed_pt_up, transformed_pt_down, transformed_pt_left, transformed_pt_right, transformed_pt_forward, transformed_pt_backward; //the neighbouring points in the LOCAL frame
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>::iterator search;
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>::iterator search_global;

	///////////////
	//Upwards Point
	///////////////
	if (pointUp.z < max_z) {
		transformed_pt_up.x = local_point.x;
		transformed_pt_up.y = local_point.y;
		transformed_pt_up.z = local_point.z  + dot_distance;
		bool y = convert_local_to_global(pointUp, transformed_pt_up, timeStamp, transformStamped_local_to_global);
		if (isPointInside(scan, transformed_pt_up)) { //if neighbouring point is still inside of scan, we just add it to the list of points to visit	
			search = visited_map_2.find(transformed_pt_up);
			search_global = visited_map.find(pointUp);
			if (search == visited_map_2.end()) { //if locally, have not visited the point yet
				++index_local;
				visited_map_2.insert({transformed_pt_up, index_local}); //add it to locally visited points
				if (search_global == visited_map.end()) {//cannnot find global point -> if we have not visited this point globally yet, we add it to geometry message
					std::cout << "global_point: "  << pointUp << std::endl;
					q.push(std::make_tuple(transformed_pt_up, pointUp, true)); //add it to the queue
					++index_global;
					visited_map.insert({pointUp, index_global}); //add it to globally visited points
				} else { //have visited this neighbouring point before
					std::cout << "GLOBALLY - FALSE Up:  have visited this neighbouring point before: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
					q.push(std::make_tuple(transformed_pt_up, pointUp, false)); //add it to the queue
				}
			} else {
				// std::cout << "LOCALLY Up: have visited this neighbouring point before: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointUp
			} else {
				e.node_ids.push_back(visited_map[pointUp]);
			}

		} else {
			// std::cout << "OUTSIDE of laserScan Up: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << std::endl;	
		}
	} else {
		// std::cout << "exceed max_z: " << pointUp.x << ", " << pointUp.y << ", " << pointUp.z << " || max_z: " << max_z << std::endl;	
	}

	///////////////
	//Downwards Point
	///////////////
	if (pointDown.z < max_z) {
		transformed_pt_down.x = local_point.x;
		transformed_pt_down.y = local_point.y;
		transformed_pt_down.z = local_point.z  - dot_distance;
		bool y = convert_local_to_global(pointDown, transformed_pt_down, timeStamp, transformStamped_local_to_global);
		if (isPointInside(scan, transformed_pt_down)) { //if neighbouring point is still inside of scan 
			search = visited_map_2.find(transformed_pt_down);
			search_global = visited_map.find(pointDown);
			if ( search == visited_map_2.end() ) { //if locally, I have not visited the point yet
				++index_local;
				visited_map_2.insert({transformed_pt_down, index_local}); //add it to locally visited points
				if (search_global == visited_map.end()) {
					// std::cout << "GLOBALLY - TRUE: pushed into queue Down: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z  << std::endl;	
					q.push(std::make_tuple(transformed_pt_down, pointDown, true));
					++index_global;
					visited_map.insert({pointDown, index_global}); //add it to globally visited points
				} else { //globally, I have visited this point before, so add the index_global of pointDown to the the edge list of the original point
					std::cout << "GLOBALLY - FALSE Down: have visited this neighbouring point before: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
					q.push(std::make_tuple(transformed_pt_down, pointDown, false));
				}
			} else {
				// std::cout << "LOCALLY Down: have visited this neighbouring point before: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointDown
			} else {
				e.node_ids.push_back(visited_map[pointDown]);
			}

		} else {
			// std::cout << "OUTSIDE of laserScan Down: " << pointDown.x << ", " << pointDown.y << ", " << pointDown.z << std::endl;	
		}
	}
	
	///////////////
	//Left Point
	///////////////
	if (pointLeft.z < max_z) {		
		transformed_pt_left.x = local_point.x;
		transformed_pt_left.y = local_point.y - dot_distance;
		transformed_pt_left.z = local_point.z;
		if (isPointInside(scan, transformed_pt_left)) { 
			convert_local_to_global(pointLeft, transformed_pt_left, timeStamp, transformStamped_local_to_global);
			// std::cout << "pointLeft global_point: "  << pointLeft << std::endl;
			search = visited_map_2.find(transformed_pt_left);
			search_global = visited_map.find(pointLeft);
			if (search == visited_map_2.end()) {
				++index_local;	
				visited_map_2.insert({transformed_pt_left, index_local}); //add it to locally visited points
				if (search_global == visited_map.end()) {
					// std::cout << "GLOBALLY - TRUE: pushed into queue Left: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;
					q.push(std::make_tuple(transformed_pt_left, pointLeft, true));
					++index_global;
					visited_map.insert({pointLeft, index_global}); //add it to visited points
				} else { //if i have visited pointLeft before
					std::cout << "GLOBALLY - FALSE Left: have visited this neighbouring point before: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
					q.push(std::make_tuple(transformed_pt_left, pointLeft, false));
				}
			} else {
				// std::cout << "OUTSIDE of laserScan Left: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointLeft
			} else {
				e.node_ids.push_back(visited_map[pointLeft]);
			}

		} else {
			// std::cout << "LOCALLY Left: have visited this neighbouring point before: " << pointLeft.x << ", " << pointLeft.y << ", " << pointLeft.z << std::endl;	
		}
	}

	
	///////////////
	//Right Point
	///////////////
	if (pointRight.z < max_z) {		
		transformed_pt_right.x = local_point.x;
		transformed_pt_right.y = local_point.y + dot_distance;
		transformed_pt_right.z = local_point.z;
		if (isPointInside(scan, transformed_pt_right)) { 
			convert_local_to_global(pointRight, transformed_pt_right, timeStamp, transformStamped_local_to_global);
			search = visited_map_2.find(transformed_pt_right);
			search_global = visited_map.find(pointRight);
			if (search == visited_map_2.end()) {
				// std::cout << "GLOBALLY - TRUE : pushed into queue Right: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;
				++index_local;	
				visited_map_2.insert({transformed_pt_right, index_local}); //add it to locally visited points
				if (search_global == visited_map.end()) {
					q.push(std::make_tuple(transformed_pt_right, pointRight, true));
					++index_global;
					visited_map.insert({pointRight, index_global}); //add it to visited points
				} else { //if i have visited pointRight before
					std::cout << "GLOBALLY - FALSE Right: have visited this neighbouring point before: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
					q.push(std::make_tuple(transformed_pt_right, pointRight, false));
				}
			} else {
				// std::cout << "LOCALLY Right: have visited this neighbouring point before: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
			} 

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointRight
			} else {
				e.node_ids.push_back(visited_map[pointRight]);
			}

		} else {
			// std::cout << "OUTSIDE of laserScan Right: " << pointRight.x << ", " << pointRight.y << ", " << pointRight.z << std::endl;	
		}
	}
	
	
	///////////////
	//Forwards Point
	///////////////
	if (pointForward.z < max_z) {		
		transformed_pt_forward.x = local_point.x + dot_distance;
		transformed_pt_forward.y = local_point.y;
		transformed_pt_forward.z = local_point.z;
		// std::cout << "5. isPointInside: "  << isPointInside(scan, transformed_pt_forward) << std::endl; 
		if (isPointInside(scan, transformed_pt_forward)) { 
			convert_local_to_global(pointForward, transformed_pt_forward, timeStamp, transformStamped_local_to_global);
			search = visited_map_2.find(transformed_pt_forward);
			search_global = visited_map.find(pointForward);
			if (search == visited_map_2.end()) {
				//std::cout << "pushed into queue Left: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;
				++index_local;	
				visited_map_2.insert({transformed_pt_forward, index_local}); //add it to locally visited points
				if (search_global == visited_map.end()) {
					// std::cout << "GLOBALLY - TRUE pushed into queue Forward: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
					q.push(std::make_tuple(transformed_pt_forward, pointForward, true));
					++index_global;
					visited_map.insert({pointForward, index_global}); //add it to visited points
				} else { //if i have visited pointForward before
					std::cout << "GLOBALLY - FALSE Forward: have visited this neighbouring point before: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
					q.push(std::make_tuple(transformed_pt_forward, pointForward, false));
				}
			} else {
				// std::cout << "LOCALLY Forward: have visited this neighbouring point before: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointForward
			} else {
				e.node_ids.push_back(visited_map[pointForward]);
			}

		} else {
			// std::cout << "OUTSIDE of laserScan Forward: " << pointForward.x << ", " << pointForward.y << ", " << pointForward.z << std::endl;	
		}
	}
	

	///////////////
	//Backwards Point
	///////////////
	if (pointBackward.z < max_z) {
		transformed_pt_backward.x = local_point.x - dot_distance;
		transformed_pt_backward.y = local_point.y;
		transformed_pt_backward.z = local_point.z;
		// std::cout << "6. isPointInside: "  << isPointInside(scan, transformed_pt_backward) << std::endl; 
		if (isPointInside(scan, transformed_pt_backward)) { //if pointBackward is still inside of scan, we add it to the list of points to visit
			convert_local_to_global(pointBackward, transformed_pt_backward, timeStamp, transformStamped_local_to_global);
			search = visited_map_2.find(transformed_pt_backward);
			search_global = visited_map.find(pointBackward);
			if (search == visited_map_2.end()) { //if locally, have not visited pointBackward yet
				++index_local;
				visited_map_2.insert({transformed_pt_backward, index_local}); //add it to locally visited points
				if (search_global == visited_map.end()) {//if we have not visited this point globally yet, we add it to geometry message
					// std::cout << "GLOBALLY - TRUE pushed into queue Backward: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
					q.push(std::make_tuple(transformed_pt_backward, pointBackward, true));
					++index_global;
					visited_map.insert({pointBackward, index_global}); //add it to globally visited points
				} else { //have visited this neighbouring point before
					std::cout << "GLOBALLY - FALSE Backward: have visited this neighbouring point before: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
					q.push(std::make_tuple(transformed_pt_backward, pointBackward, false));
				}
			} else {
					// std::cout << "LOCALLY Backward: have visited this neighbouring point before: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;	
			}

			if (search_global == visited_map.end()) {//as long as we this point is a valid point, we need to add to the main point's neighbours. if we have not visited this point globally yet
				e.node_ids.push_back(index_global); //index_global should be referring to pointBackward
			} else {
				e.node_ids.push_back(visited_map[pointBackward]);
			}

		} else {
			// std::cout << "OUTSIDE of laserScan Backward: " << pointBackward.x << ", " << pointBackward.y << ", " << pointBackward.z << std::endl;
		}
	}
}



void cloud_cb(ros::Time& timeStamp, pcl::PointCloud<pcl::PointXYZ>& original_cloud_msg) { //pcl::PointCloud<pcl::PointXYZ>& original_cloud_msg, 
	int replace_index_local = 1;

	geometry_msgs::TransformStamped transformStamped_local_to_global;
	try {
		transformStamped_local_to_global = tfBuffer.lookupTransform(world_frame, "X1/base_link/front_laser", timeStamp); //ros::Time(0));//timeStamp);  //tfBuffer.lookupTransform(destFrame, originFrame, ... )
	} catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
		ros::Duration(1.0).sleep();
		return;
	}

	pcl::PointXYZ start_global;
	pcl::PointXYZ local_init_point = pcl::PointXYZ(0.0,0.0,0.0);
	if (!convert_local_to_global(start_global, local_init_point, timeStamp, transformStamped_local_to_global)) { //should always be true as line 369 - 403 would have returned otherwise
		std::cout << "transform not ready..." << std::endl;
		return;
	}

	if ((original_cloud_msg.size() < 1)) { 
		std::cout << "Returning at start: !origin_is_updated: " << !origin_is_updated << " || original_cloud_msg.size(): " << original_cloud_msg.size() << "\n";
		return;
	}
	clock_t tStart = clock();
	
	//create the LidarScan from PointCloud
	LidarScan scan;
    decomposeLidarScanIntoPlanes(original_cloud_msg, scan);
	for (int i = 0; i < scan.size(); i++) {
		fillGaps(scan[i].scan, fill_gap_amt);
	}
	
	std::tuple<pcl::PointXYZ, pcl::PointXYZ, bool> temp_tuple; //declare a tuple - to keep the tuple I'll dequeue later
	pcl::PointXYZ temp_global, temp_local; //declare a point - to keep the point from within the de-queued tuple
	bool to_add; //declare a boolean - to keep the boolean value from within the de-queued tuple
	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal> visited_map_2; //per call back - to keep track of my locally visited BFS points, etc
	
	
	int index_local = 0;

	// initialise a zero XYZ point - for local
	pcl::PointXYZ start_local;
	start_local.x = 0; 
	start_local.y = 0;
	start_local.z = 0;

	// initialisation of the queue
	std::queue<std::tuple<pcl::PointXYZ, pcl::PointXYZ, bool>> q; //declare the queue of tuples - if true then add to geo msg. true/false is set in the above method. <local, global, to_add>

	std::unordered_map<pcl::PointXYZ, int, key_hash, key_equal>::iterator search = visited_map.find(start_global); //find the origin of the robot inside of the gobal_map
	if (search == visited_map.end()) { //check if origin of robot is inside visited_map. if yes, add it visited_map_2 only and is false. if no, add it to both and is true
		// ++index_local;
		++index_global;
		visited_map.insert({start_global, index_global}); //add it to visited points
		visited_map_2.insert({start_local, replace_index_local}); //add it to visited points
		q.push(std::make_tuple(start_local, start_global, true)); 
	} else {	
		// ++index_local;
		visited_map_2.insert({start_local, replace_index_local}); //add it to visited points
		q.push(std::make_tuple(start_local, start_global, false)); 
	}
	bool debug_bool = true;

	// running BFS
	while (!q.empty()) {
		
		temp_tuple = q.front();
		q.pop();
		temp_local = std::get<0>(temp_tuple); 
		temp_global = std::get<1>(temp_tuple);
		to_add = std::get<2>(temp_tuple);

		if (to_add) { //check if true or false from the tuple
			geometry_msgs::Point out_cloud_2_point; // the individual point within the visualisation
			out_cloud_2_point.x = temp_global.x;
			out_cloud_2_point.y = temp_global.y;
			out_cloud_2_point.z = temp_global.z;
			out_cloud_2.nodes.push_back(out_cloud_2_point);
		}
		
		graph_msgs::Edges e;
		add_to_queue(transformStamped_local_to_global, temp_local, q, scan, e, index_global, visited_map_2, index_local, timeStamp, debug_bool); //finds out the neighbours of the de-queued point
		debug_bool = false;

		if (to_add) {
			out_cloud_2.edges.push_back(e);	
		}
		if (index_local > 100000) {
			std::cout << "INDEX_LOCAL EXCEEDED" << "\n";
			break;
		}
		
	}
	std::cout << "out_cloud_2.nodes.size(): " << out_cloud_2.nodes.size() << std::endl;
	std::vector<uint8_t> arr(out_cloud_2.nodes.size());
	out_cloud_2.explored = arr;
	
	out_cloud_2.header.frame_id = world_frame;
	out_cloud_2.header.stamp = pcl_conversions::fromPCL(original_cloud_msg.header.stamp);
	pub2.publish(out_cloud_2);

}

void transform_the_cloud(const sensor_msgs::PointCloud2& cloud_msg) { //called when there is a new lidar scan in the form of a point cloud being sent in 
	// if (disable_skipping || current_frame >= every_x_frames ) {
	// 	current_frame = 0;
	// } else {
	// 	current_frame += 1;
	// 	std::cout << "returning due SKIP" << std::endl;
	// 	return;
	// }

	if (to_skip) {
		return;
	}

	//convert the original cloud for use in cloud_cb
	pcl::PointCloud<pcl::PointXYZ> original_cloud_msg;
	pcl_conversions::toPCL(cloud_msg, pcl_cloud_msg_2);
	pcl::fromPCLPointCloud2( pcl_cloud_msg_2, original_cloud_msg); 

	//call the grapher method
	ros::Time timeStamp = cloud_msg.header.stamp;
	cloud_cb(timeStamp, original_cloud_msg);
	
}

void to_skip_or_not_to_skip(const sensor_msgs::Imu& imu_msg) {
	if (abs(imu_msg.angular_velocity.z) > 0.15 || abs(imu_msg.angular_velocity.x) > 0.1 || abs(imu_msg.angular_velocity.y) > 0.1) {
		to_skip = true;
	} else {
		if (to_skip) {
			ros::Duration(1.5).sleep();
			to_skip = false;
		}
	}
}

int main (int argc, char* argv[]) {
	ros::init (argc, argv, "dots_node");
	ros::NodeHandle nh;
  	ros::NodeHandle nhPrivate = ros::NodeHandle("~");

	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Subscriber sub = nh.subscribe ("/X1/points/", 1, transform_the_cloud);
	ros::Subscriber sub2 = nh.subscribe ("/X1/imu/data", 1, to_skip_or_not_to_skip);

	std::string output_topic = "output_2";
	pub2 = nh.advertise<graph_msgs::GeometryGraph> (output_topic, 1);	
	pub3 = nh.advertise<sensor_msgs::PointCloud2> ("transformed_point_cloud", 1);	
	
	nhPrivate.getParam("world_frame", world_frame);
	// std::cout << "world frame: " << world_frame << std::endl;

	ros::spin();
}