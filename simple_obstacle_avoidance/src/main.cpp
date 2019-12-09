#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#define ROBOT_WIDTH 0.3
#define ROBOT_FRONT 0.15
#define ROBOT_BACK -0.47
#define ROBOT_WIDTH_NEW 0.5
#define MAX_SPEED 0.1
#define DELAY 2.0
enum Direction  {
	FORWARD = 0, LEFT = 1, RIGHT = 2, BACKWARD = 3
};

enum MotionState {
        SPOT_TURN= 0,  STRAIGHT = 1
};

MotionState mstate = STRAIGHT;

sensor_msgs::LaserScan distanceTraversible(sensor_msgs::LaserScan scan, double robot_width) {
	double curr_angle = scan.range_min;
	double scan_increment = scan.angle_increment;
	
	std::vector<geometry_msgs::Point> points;
	
	for(int i = 0; i < scan.ranges.size(); i++) {
		float range  = scan.ranges[i];
		if(range > scan.range_max)
			continue;
		geometry_msgs::Point point;
		point.x = range*cos(curr_angle);
		point.y = range*sin(curr_angle);
		points.push_back(point);
		curr_angle += scan_increment;
	}

	sensor_msgs::LaserScan traversibility;
	traversibility.angle_increment = scan.angle_increment;
	traversibility.angle_increment = scan.angle_min;
	traversibility.angle_max = scan.angle_max;
	traversibility.header = scan.header;
	traversibility.scan_time = scan.scan_time;
	traversibility.time_increment = scan.time_increment;
	traversibility.range_min = scan.range_min;
	traversibility.range_max = scan.range_max;
 	float angle = scan.range_min;
	
	for(int i = 0; i < scan.ranges.size(); i++) {

		float min_dist = std::numeric_limits<float>::infinity();
		
		geometry_msgs::Point robot_left;
		geometry_msgs::Point robot_right;

		robot_left.x = -robot_width/2*cos(angle);
		robot_left.y = -robot_width/2*sin(angle);

		robot_right.x = robot_width/2*cos(angle+M_PI);
		robot_right.y = robot_width/2*sin(angle+M_PI);

		for(geometry_msgs::Point point : points) {

			float dx1 = -(point.x - robot_right.x);
			float dy1 = -(point.y - robot_right.y);

			float dx2 = -(point.x - robot_left.x);
			float dy2 = -(point.y - robot_left.y);

			float dot_product = dx1*dx2 + dy1*dy2;
			float radius = sqrt(point.x*point.x + point.y*point.y);

			std::cout << dot_product << ", " << radius << ", "<< point.x << ", " << point.y << "," << (dot_product > robot_width || dot_product < 0) << ", " << angle <<std::endl;

			

			if(dot_product > robot_width || dot_product < 0)
				continue;
			
			if(min_dist > radius) {
				min_dist = radius;
				std::cout << "radius "<< radius << std::endl;
			}
			
		}
		std::cout << "got " << min_dist << std::endl;
		traversibility.ranges.push_back(min_dist);
		traversibility.intensities.push_back(47);
		angle += scan.angle_increment;
	}

	return traversibility;
}

class LaserScanObstacles {
	bool obstacles[4];
	bool updating = false;
public:
	LaserScanObstacles(sensor_msgs::LaserScan scan) {
		updating = true;
		for(int i = 0; i < 4; i++){
			obstacles[i] = false;
		}
		float currAngle = scan.angle_min;
		for(int  i = 0 ; i < scan.ranges.size(); i++) {
			float radius = scan.ranges[i];
			currAngle += scan.angle_increment;

			float x_pos = radius*cos(currAngle);
			float y_pos = radius*sin(currAngle);

			if(y_pos < ROBOT_WIDTH && y_pos > -ROBOT_WIDTH && radius < 1) {
				if(x_pos < 0)
					obstacles[FORWARD] = true;
				else
					obstacles[BACKWARD] = true;
			}
			else if(y_pos < 0 && x_pos < ROBOT_FRONT && x_pos > ROBOT_BACK ) {
				obstacles[LEFT] = true;
			}
			else if(y_pos > 0 && x_pos < ROBOT_FRONT && x_pos > ROBOT_BACK ) {
				obstacles[RIGHT] = true;
			}
		}
		updating = false;
	}
	bool containsObstacle(Direction direction){
		while(updating) {}
		return obstacles[direction];
	}

	void debugPrint() {
		std::cout << "Front : " << containsObstacle(FORWARD)
				<< "Left : " << containsObstacle(LEFT)
				<< "Right : " << containsObstacle(RIGHT)
				<< "Back : " << containsObstacle(BACKWARD)
				<< std::endl;
	}
};

enum State {
	OBSTACLE_SEARCH, WALL_FOLLOW
};

class SimpleObstacleAvoidance {
public:
	ros::NodeHandle nh;
	ros::Publisher velocityController;
	ros::Publisher traversibilityMap;
	ros::Subscriber laserScanSub;
	ros::Time changeTriggered;
	State state;


	void obstacleSearch(LaserScanObstacles& obstacles) {
		if(obstacles.containsObstacle(FORWARD)) {
			//turn right
			mstate = SPOT_TURN;
			geometry_msgs::Twist twist;
			twist.linear.x = 0;
			twist.angular.z = -0.3;
			velocityController.publish(twist);
			std::cout << "searching for obstacle" << std::endl;
		}
		else if(obstacles.containsObstacle(LEFT)) {
			std::cout << "switching state" << std::endl;
			state = WALL_FOLLOW; 
		}
		else {
			mstate = STRAIGHT;
			std::cout << "searching for obstacle" << std::endl;
			geometry_msgs::Twist twist;
			twist.linear.x = 0.1;
			twist.angular.z = 0;
			velocityController.publish(twist);
		}
	}
	ros::Time last_turn;
	bool turn_enable = false;
	void wallFollow(LaserScanObstacles& obstacles) {
		if((ros::Time::now() - last_turn).sec < 5 && turn_enable == true) {
			return;
		}
		turn_enable = false;
		if(!obstacles.containsObstacle(LEFT)) {
			last_turn =ros::Time::now();
			turn_enable = true;
			// turn left if the left is free
			geometry_msgs::Twist twist;
			mstate = SPOT_TURN;
			twist.linear.x = 0;
			twist.angular.z = 0.3;
			velocityController.publish(twist);
			std::cout << "Turning left - no obstacle on left" << std::endl;
		} else if (!obstacles.containsObstacle(FORWARD)) {
			geometry_msgs::Twist twist;
			mstate = STRAIGHT;
			twist.linear.x = 0.1;
			twist.angular.z = 0;
			velocityController.publish(twist);
			std::cout << "Following wall" << std::endl;
		} else {
			//turn right if left and right are obstructed.
			geometry_msgs::Twist twist;
			mstate = SPOT_TURN;
			twist.linear.x = 0;
			twist.angular.z = -0.3;
			velocityController.publish(twist);
			std::cout << "Evasive maneuver" << std::endl;

		}
	}
	void onLaserScan(sensor_msgs::LaserScan scan) {
		LaserScanObstacles obsmap(scan);
		traversibilityMap.publish(distanceTraversible(scan, ROBOT_WIDTH_NEW));
		std::cout << "current state " << state << std::endl;
		if(state == OBSTACLE_SEARCH)
			obstacleSearch(obsmap);
		else if(state == WALL_FOLLOW)
			wallFollow(obsmap);
		obsmap.debugPrint();
	}

	SimpleObstacleAvoidance(ros::NodeHandle nh) {
		state = OBSTACLE_SEARCH;
		velocityController = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
		laserScanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &SimpleObstacleAvoidance::onLaserScan, this);
		traversibilityMap = nh.advertise<sensor_msgs::LaserScan>("/out/scan", 10);
	}

};
int main(int argc, char** argv) {
	ros::init(argc, argv, "SimpleObsAvoid");
	ros::NodeHandle n;
	SimpleObstacleAvoidance s(n);
	ros::spin();
}
