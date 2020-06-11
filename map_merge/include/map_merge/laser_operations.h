#ifndef _LASER_OPERATIONS_H_
#define _LASER_OPERATIONS_H_

#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

sensor_msgs::LaserScan toLaserScan(pcl::PointCloud<pcl::PointXYZ>& pc);

float* lookup(sensor_msgs::LaserScan& scan, int index); 

/**
 * Performs linear interpolation of lasercans
 */ 
void fillGaps(sensor_msgs::LaserScan& scan, size_t max_gap=2);

/**
 * Naive corner detector
 */
void naiveCornerDetector(sensor_msgs::LaserScan& scan, pcl::PointCloud<pcl::PointXYZ>& corners, std::vector<int>& indices, int skip=1);

/**
 * Normalize centroid of laserscan
 */
void centroidNormalization(const sensor_msgs::LaserScan& scan, sensor_msgs::LaserScan& recalculate, float resolution); 
#endif