#ifndef _PHY_PLANNER_CYL_COORDS_H_
#define _PHY_PLANNER_CYL_COORDS_H_

#define tf_vec_debug(label , v) std::cout <<label << v.x() << ", " << v.y() << ", " <<v.z() <<std::endl;
/**
 * (c) Arjo Chakravarty 2020-
 * 
 * This file contains interfaces to a set of core utilities to handle rotations as described in the cylindrical coordinate system
 */
#include <vector>
#include <tf/tf.h>
#include <amapper/elevation_grid.h>

/**
 * A contact area describes a certain part of the robot body. Currently a contact area consists of points 
 * in a vector that are sampled at a regular interval.
 */ 
typedef std::vector<tf::Point> ContactArea;

/**
 * Describes a cylindrical coordinate frame in relation to a cartesian axis
 */ 
struct CylindricalCoordinateFrame {
    tf::Vector3 origin; /// The origin of the cylindrical coordinate system in relation to the cartesian coordinate system
    tf::Vector3 axis; /// The axis of rotation. Theta is measured along this axis.
    tf::Vector3 planar_axis; /// The axis which we consider theta=0. Should be perpendicular to `axis`.
};

/**
 * Describes a pivot point
 */ 
struct TumblePoint {
    int  count; ///Number of supporting pivots
    CylindricalCoordinateFrame axis; ///The axis along which the pivot will occur
};


/***
 * Given the center of mass of a vehicle, determines the direction in which the vehicle will pivot
 * @param center_of_mass is where the center of mass of the vehicle is
 * @param first_contact is where the first contact is made
 * @param current_normal is the normal vector in relation to the robot
 */ 
CylindricalCoordinateFrame getPivotFromSinglePoint(tf::Vector3 center_of_mass, tf::Vector3 first_contact, tf::Vector3 current_normal);

/**
 * Calculates the projection of a vector
 */ 
double projection(tf::Vector3 vec, tf::Vector3 onto);

/**
 * Checks if a point is inside a line segment
 */ 
TumblePoint getTumbleAxis(tf::Vector3 start_lineseg, tf::Vector3 end_lineseg, tf::Vector3 point, tf::Vector3 normal=tf::Vector3(0,0,1));

/**
 * Gets sample points in a rectangle
 */  
ContactArea rectangularSample(float width, float height, float sample_interval, float default_height=0);

/**
 * Transforms sample points
 */ 
void transformPoints(ContactArea& sample_points, tf::Transform target_pose);

/**
 * Combines contact areas
 */ 
ContactArea combineContactArea(std::vector<ContactArea> contactAreas);

/**
 * Extracts relevant amapper elevation points. Transforms the amapper point to local coordinates.
 */ 
std::vector<tf::Vector3> transformElevGridToLocalFrame(ContactArea ca, AMapper::ElevationGrid grid, geometry_msgs::Pose pose);

typedef AMapper::GenericCoordinateStorage<std::vector<float>> CylindricalMap;

void add(CylindricalMap& map, float r, float z, float theta) {
    auto _r = (long)(r*10.0f);
    auto _z = (long)(z*10.0f);
    auto coordinate = std::make_pair(_r,_z);
    map[coordinate].push_back(theta);
}

std::vector<float> find(CylindricalMap& map, float r, float z) {
    auto _r = (long)(r*10.0f);
    auto _z = (long)(z*10.0f);
    auto coordinate = std::make_pair(_r,_z);
    //std::cout << "query " << r << "->"<< _r << ":" << z << "-> "<< _z  <<std::endl;
    return map[coordinate];
}
/**
 * Determine first contact point in a cylindrical transformed coordinate system.
 * @returns the cylindrical coordinate system
 */ 
tf::Point getClosestCylindrical(ContactArea cylindrical, ContactArea footprint) {
    
    CylindricalMap map; 

    for(auto pt: cylindrical) {
        add(map, pt.x(), pt.y(), pt.z());
    }

    tf::Vector3 min_pt(0, 0, INFINITY);
    for(auto pt: footprint) {
        //tf_vec_debug("point", pt);
        for(auto t :  find(map, pt.x() , pt.y())){
            auto angle = pt.z() - t;
            if(angle < min_pt.z()) {
                min_pt = tf::Vector3(pt.x(), pt.y(), angle);
            }
        }
    }
    return min_pt;
}

struct TumbleResult{
    bool ok;
    float angle;
    tf::Vector3 intersectionPoint;
};
/**
 * Performs the actual tumble
 */ 
TumbleResult executeTumble(TumblePoint pt, ContactArea terrain, ContactArea footprint) {

    auto cylindricalTerrain = transformToCylindrical(terrain, pt.axis.axis, pt.axis.planar_axis, pt.axis.origin);
    auto cylindricalFootprint = transformToCylindrical(footprint, pt.axis.axis, pt.axis.planar_axis, pt.axis.origin);
    auto result = getClosestCylindrical(cylindricalTerrain, cylindricalFootprint);
    TumbleResult tr;
    tr.angle = result.z();
    tr.ok = false;
    tr.intersectionPoint = cylindricalToCartesian(result, pt.axis.axis, pt.axis.planar_axis, pt.axis.origin);
    if(tr.angle < 1.0) {
        tr.ok = true;
    }
    return tr;
}

#endif