#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <cmath>
#define ROBOT_WIDTH 0.3
#define ROBOT_FRONT 0.15
#define ROBOT_BACK -0.47
#define MAX_SPEED 0.1
#define DELAY 2.0
#define BASE_LINK "husky1/base_link"

enum Direction  {
	FORWARD = 0, LEFT = 1, RIGHT = 2, BACKWARD = 3
};

enum MotionState {
        SPOT_TURN= 0,  STRAIGHT = 1
};
MotionState mstate = STRAIGHT;
class LaserScanObstacles {
	bool obstacles[4];
	bool updating = false;
public:
	float left_wall_radius;
	float front_dist;
	bool is_left_turn_possible;
	LaserScanObstacles(sensor_msgs::LaserScan scan, tf::TransformListener& listener) {
		updating = true;
		is_left_turn_possible = false;
		for(int i = 0; i < 4; i++){
			obstacles[i] = false;
		}
		float currAngle = scan.angle_min;
		float left_opening_start_x, left_opening_start_y;
		bool wasOpen = false;
		float maxAngleRadius = -1;
		float maxAngle = 1;
		for(int  i = 0 ; i < scan.ranges.size(); i++) {
			float radius = scan.ranges[i];
			currAngle += scan.angle_increment;
			//std::cout << radius <<std::endl;
			if(-1.58 < currAngle && currAngle < -1.57) {
				left_wall_radius =  radius;
			}
			if(currAngle > 0.01 && currAngle < 0.01) {
				front_dist = radius;
			}
			float x_pos = radius*cos(currAngle);
			float y_pos = radius*sin(currAngle);
			
			tf::Vector3 pt(x_pos, y_pos, 0);
			tf::Stamped<tf::Vector3> out;
			tf::Stamped<tf::Vector3> stampedPt(pt, scan.header.stamp, scan.header.frame_id);
			listener.transformPoint(BASE_LINK, stampedPt, out);

			x_pos = out.x();
			y_pos = out.y();

			if(y_pos < ROBOT_WIDTH && y_pos > -ROBOT_WIDTH && radius < 1) {
				if(x_pos > 0)
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
			
			/*if(radius > 1 && currAngle < 0 && currAngle > -1.57&& !wasOpen) {
				wasOpen = true;
				std::cout << "starting at " << currAngle << ","<< radius <<std::endl;
				left_opening_start_x = x_pos; 
				
				left_opening_start_y = y_pos;
			}
			if(wasOpen && (radius < 1 || currAngle > 0)) {
				std::cout << "locking at " << currAngle << ","<< radius <<std::endl;
				float dist_x = x_pos - left_opening_start_x;
				float dist_y = y_pos - left_opening_start_y;
				if(dist_x*dist_x +dist_y*dist_y > 0.5) {
					is_left_turn_possible = true;
				}
				wasOpen = false;
			}*/
			if(maxAngleRadius < radius &&currAngle < -1.57 && currAngle > -3.1415 && radius <  30.0f ) {
				maxAngleRadius = radius;
				maxAngle = currAngle;
				std::cout << "got radius" << radius <<std::endl;	
			}
		}
		std::cout << maxAngle<<","<< maxAngleRadius<<std::endl;
		is_left_turn_possible = (maxAngle < 0 && maxAngleRadius > 0.5);
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
				<< " can I turn left?" << is_left_turn_possible 
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
	ros::Subscriber laserScanSub;
	ros::Subscriber enabled;
	ros::Time changeTriggered;
	tf::TransformListener* listener;
	bool stop = false;
	State state;



	void obstacleSearch(LaserScanObstacles& obstacles) {
		if(obstacles.containsObstacle(FORWARD)) {
			//turn right
			mstate = SPOT_TURN;
			geometry_msgs::Twist twist;
			twist.linear.x = 0;
			twist.angular.z = -0.1;
			velocityController.publish(twist);
			std::cout << "searching for obstacle" << std::endl;
		}
		else if(obstacles.containsObstacle(LEFT)) {
			std::cout << "switching state" << std::endl;
			state = WALL_FOLLOW; 
		}
		else {
			mstate = STRAIGHT;
			std::cout << "searchin for obstacle" << std::endl;
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
			twist.angular.z = 0.1;
			velocityController.publish(twist);
			std::cout << "Turning left - no obstacle on left" << std::endl;
		} else if (!obstacles.containsObstacle(FORWARD)) {
			std::cout << "" << std::endl;
			geometry_msgs::Twist twist;
			mstate = STRAIGHT;
			twist.linear.x = 0.1;
			twist.angular.z = 0;
			if(obstacles.left_wall_radius > 0.7)
				twist.angular.z = 0.1;
			else if(obstacles.left_wall_radius <0.5)
				twist.angular.z = -0.1;
			velocityController.publish(twist);
			std::cout << "Following wall" << std::endl;
		} else if (obstacles.front_dist < 1){
			//turn right if left and right are obstructed.
			geometry_msgs::Twist twist;
			mstate = SPOT_TURN;
			twist.linear.x = 0;
			//if(!obstacles.is_left_turn_possible)
			twist.angular.z = -0.3;
			velocityController.publish(twist);
			/*else
			twist.angular.z = 0.1;
			elocityController.publish(twist);
			std::cout << "Evasive maneuver" << std::endl;*/

		}
	}
	void onLaserScan(sensor_msgs::LaserScan scan) {
		if(stop){
			geometry_msgs::Twist twist;
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.angular.z = 0;
			velocityController.publish(twist);
			return;
		}
		LaserScanObstacles obsmap(scan, *listener);
		std::cout << "current state " << state << std::endl;
		if(state == OBSTACLE_SEARCH)
			obstacleSearch(obsmap);
		else if(state == WALL_FOLLOW)
			wallFollow(obsmap);
		obsmap.debugPrint();
	}

	void onEStop(std_msgs::String msg) {
		//STOP THE MACHINi
		stop = true;
		std::cout << "ESTOP ACTIVATED" << std::endl;
	}

	SimpleObstacleAvoidance(ros::NodeHandle nh) {
		state = OBSTACLE_SEARCH;
		velocityController = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
		laserScanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &SimpleObstacleAvoidance::onLaserScan, this);
		enabled = nh.subscribe<std_msgs::String>("e_stop", 10, &SimpleObstacleAvoidance::onEStop, this);
		listener = new tf::TransformListener();
	}

};
int main(int argc, char** argv) {
	ros::init(argc, argv, "SimpleObsAvoid");
	ros::NodeHandle n;
	SimpleObstacleAvoidance s(n);
	ros::spin();
}
