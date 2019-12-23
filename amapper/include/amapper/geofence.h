#ifndef _AMAPPER_GEOFENCE_
#define _AMAPPER_GEOFENCE_

#include <eigen3/Eigen/Eigen>
#include <tf/transform_listener.h>
#include <vector>

struct GeoFence {
    std::vector<Eigen::Vector3f> fence;
};

GeoFence lookupGeoFence(tf::TransformListener& listener);
bool pointInsideGeoFence(GeoFence geofence, geometry_msgs::Point point);

#endif