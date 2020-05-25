#ifndef _ROBOT_MODEL_H_
#define _ROBOT_MODEL_H_
#include "cylindrical_coordinates.h"

struct FinalPose {
    float z;
    bool ok;
    tf::Quaternion quaternion;
};
/**
 * Builds a robot model 
 * Robot may be made of "Good" (wheels/track) and "Bad" contact points (body)
 * In this example we will model a robot as follows:
 * 
 *   []------[]
 *     |    |
 *     |    |
 *   []------[]
 * 
 * Body will be 0.5*0.5m it is raised 0.1m above the ground.
 * Wheels will be 0.1*0.1m mounted at the ends of the body.
 */ 
class RobotModel {
    void makeWheel(std::string name, double x, double y);
public:
    std::unordered_map<std::string, ContactArea> contactAreas;
    std::unordered_map<std::string, double> elevation;
    std::unordered_map<std::string, tf::Vector3> alreadyMadeContact;

    RobotModel();
    
    /**
     * Determines the first point of contact. This is done by finding the maximum height 
     */ 
    int determineFirstContactPoint(AMapper::ElevationGrid grid, geometry_msgs::Pose pose);
    
    /**
     * Get the robots footprint
     */ 
    ContactArea getRobotFootprint();

    /**
     * Determines the coordinate in which the robot will tumble
     */ 
    TumblePoint determineAxisOfTumble(tf::Vector3 currentAxis = tf::Vector3(0,0,1), tf::Vector3 com = tf::Vector3(0,0,0));

    TumbleResult secondContactPoint(AMapper::ElevationGrid grid, TumblePoint firstTumble, geometry_msgs::Pose pose);

    TumbleResult thirdContactPoint(AMapper::ElevationGrid grid, TumblePoint firstTumble, TumbleResult amount, TumblePoint finalTumble, geometry_msgs::Pose pose);

    TumbleResult tumbleAlong(AMapper::ElevationGrid grid, TumblePoint firstTumble, geometry_msgs::Pose pose, std::unordered_map<std::string, ContactArea>& transformedPoints);


    tf::Vector3 getCenterOfMass();

    tf::Quaternion determineRotation();

    FinalPose tumble(AMapper::ElevationGrid elevationMap, geometry_msgs::Pose pose);
};
#endif