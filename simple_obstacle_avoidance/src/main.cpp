#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#define ROBOT_WIDTH 0.3
#define MAX_SPEED 0.1

enum Direction  {
	FORWARD, LEFT, RIGHT, BACKWARD, STUCK
};

class SimpleObstacleAvoidance {
public:
	ros::NodeHandle nh;
	ros::Publisher velocityController;
	ros::Subscriber laserScanSub;
	Direction directionOfTraversal;

	void switchDirection() {
		switch(directionOfTraversal) {
			case FORWARD:
				directionOfTraversal = RIGHT;
				break;
			case RIGHT:
				directionOfTraversal = BACKWARD;
				break;
			case BACKWARD:
				directionOfTraversal = LEFT;
				break;
			case LEFT:
				directionOfTraversal = FORWARD;
				break;
		}
	}

	bool isSafeToGoForward(float x, float y) {
		if(isinf(x) || isinf(y)){
			return true;
		}
		switch(directionOfTraversal) {
			case FORWARD:
				return !(x > 0 && x < 1 && y < ROBOT_WIDTH && y > -ROBOT_WIDTH);
			case RIGHT:
				return !(y < 0 && y > -1 && x < ROBOT_WIDTH && x > -ROBOT_WIDTH);
			case BACKWARD:
				return !(x < 0 && x > -1 && y < ROBOT_WIDTH && y > -ROBOT_WIDTH);;
			case LEFT:
				return !(y > 0 && y < 1 && x < ROBOT_WIDTH && x > -ROBOT_WIDTH);;
		}
	}

	void onLaserScan(sensor_msgs::LaserScan scan) {
		float currAngle = scan.angle_min;
		for(int  i = 0 ; i < scan.ranges.size(); i++) {
			float radius = scan.ranges[i];
			float x_pos = radius*cos(currAngle);
			float y_pos = radius*sin(currAngle);
			if(!isSafeToGoForward(x_pos, y_pos)){
				switchDirection();
				continue;
			}
			currAngle += scan.angle_increment;
		}
		publishTwist();
	}

	void publishTwist() {
		geometry_msgs::Twist twist;
		switch (directionOfTraversal)
		{
		case FORWARD:
			twist.linear.x = MAX_SPEED;
			break;

		case BACKWARD:
			twist.linear.x = -MAX_SPEED;
			break;

		case LEFT:
			twist.linear.y = MAX_SPEED;
			break;

		case RIGHT:
			twist.linear.y = -MAX_SPEED;
			break;
		
		default:
			break;
		}
		velocityController.publish(twist);
	}

	SimpleObstacleAvoidance(ros::NodeHandle nh) {
		velocityController = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
		laserScanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &SimpleObstacleAvoidance::onLaserScan, this);
	}

};
int main(int argc, char** argv) {
	ros::init(argc, argv, "SimpleObsAvoid");
	ros::NodeHandle n;
	SimpleObstacleAvoidance s(n);
	ros::spin();
}
