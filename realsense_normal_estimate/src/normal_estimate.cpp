#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <amapper/grid.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#define GRID_CELL_WIDTH 15 
#define GRID_CELL_HEIGHT 50// number of grids in the map. WIDTH x HEIGHT cell map
#define GRID_RESOLUTION 0.1 // unit is meter/cell. 
#define LOWEST_GRID_DEPTH -5.0 // Ignore points below this depth (-inf). In meters
#define GROUND_DEPTH_OFFSET 0.50 // How high off the ground is the Realsense
#define FORWARD_DEADZONE 0.60 // Forward Distance less than which Realsense cannot report coordinates

#define GRID_METRIC_HEIGHT GRID_CELL_HEIGHT * GRID_RESOLUTION // Grid Height in meters
#define GRID_METRIC_WIDTH GRID_CELL_WIDTH * GRID_RESOLUTION

// convert from realsense optical frame of x right, y down, z forward (EAST, DOWN, NORTH) to ROS ENU
void convertOpticalToEnuFrame(cv::Point3d& pt) {
    // Ideally should perform using transforms
    double x = pt.x;
    double y = pt.y;
    double z = pt.z;
    pt.x = x;
    pt.y = z;
    pt.z = -y;
}


class NormalEstimator {
public:
    NormalEstimator();
private:
    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    ros::Publisher grid_pub, pcl_pub, plane_normal_pub;
    AMapper::Grid amapper_grid_map;
    std::vector<std::vector<double>> depth_map;
    std::vector<std::vector<std::vector<double>>> intermediate_depth_map;
    void onPCLReceived(const pcl::PointCloud<pcl::PointXYZ> & pcl_msg);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> bestPlaneFromPoints(const std::vector<Eigen::Vector3d> & c);
    void set2dGridMap();
    void publishPclFromDepthMap();
    void publishPlaneNormal(std::pair<Eigen::Vector3d, Eigen::Vector3d> plane_normal_properties);
    void clearMapData();
};

NormalEstimator::NormalEstimator(): amapper_grid_map(0, GRID_METRIC_HEIGHT / 2.0 + FORWARD_DEADZONE, GRID_CELL_WIDTH, 
                                    GRID_CELL_HEIGHT, GRID_RESOLUTION), 
                                    intermediate_depth_map(GRID_CELL_HEIGHT, std::vector< 
                                    std::vector<double> > (GRID_CELL_WIDTH, std::vector<double>(1, LOWEST_GRID_DEPTH))),
                                    depth_map(GRID_CELL_HEIGHT, std::vector<double>(GRID_CELL_WIDTH, LOWEST_GRID_DEPTH)) {
    pcl_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &NormalEstimator::onPCLReceived, this);
    grid_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/plane_segmentation/grid_map", 1);
    pcl_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/plane_segmentation/points", 1);
    plane_normal_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/plane_segmentation/normal_plane_debug", 1);
    amapper_grid_map.setFrameId("camera_link");
    ROS_INFO("Starting 3D Map Node with parameters: \nGRID_METRIC_SIZE: %f x %f meters\nGRID_CELLS: %d x %d cells\nForward Offset = %f", 
                    GRID_CELL_WIDTH*GRID_RESOLUTION, GRID_CELL_HEIGHT*GRID_RESOLUTION, GRID_CELL_WIDTH, GRID_CELL_HEIGHT, FORWARD_DEADZONE);
}

// inspiration from https://gist.github.com/ialhashim/0a2554076a6cf32831ca
// Least Squares + SVD for closest plane fit. Returns (plane_center, plane_normal)
std::pair<Eigen::Vector3d, Eigen::Vector3d> NormalEstimator::bestPlaneFromPoints(const std::vector<Eigen::Vector3d> & c) {
	size_t num_atoms = c.size();
	Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
	for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

	// calculate centroid
	Eigen::Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

	// subtract centroid
	coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

	// we only need the left-singular matrix here
	//  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
	auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Vector3d plane_normal = svd.matrixU().rightCols<1>();
    return std::make_pair(centroid, plane_normal);
}

void NormalEstimator::publishPlaneNormal(std::pair<Eigen::Vector3d, Eigen::Vector3d> plane_normal_properties) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
    
    Eigen::Vector3d plane_center = plane_normal_properties.first;
    Eigen::Vector3d plane_normal = plane_normal_properties.second;

    int max_points_to_draw = 50;
    float point_resolution = 0.05; // gap between points in meters
    Eigen::Vector3d start_point = plane_center - plane_normal*(max_points_to_draw / 2.0) * point_resolution;
    Eigen::Vector3d point = start_point;

    for (int idx=0;idx<max_points_to_draw;idx++) {
        msg->points.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        point += plane_normal * point_resolution;
    } 

    msg->header.frame_id = "camera_link";
    msg->height = 1;
    msg->width = max_points_to_draw;

    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    plane_normal_pub.publish(msg);
}

// Reset the amapper grid and 2 other depth map data structures before next PCL data comes in
void NormalEstimator::clearMapData() {
    for (int x_idx=0;x_idx<GRID_CELL_WIDTH;x_idx++) {
        for (int y_idx=0;y_idx<GRID_CELL_HEIGHT;y_idx++) {
            depth_map[y_idx][x_idx] = 0;
        }
    }
    amapper_grid_map.clear();
    intermediate_depth_map.clear();
    std::vector<std::vector<std::vector<double>>> temp(GRID_CELL_HEIGHT, std::vector< 
                                    std::vector<double> > (GRID_CELL_WIDTH, std::vector<double>(1, LOWEST_GRID_DEPTH) ));
    intermediate_depth_map = temp;
}

// Set the 2d depth map grid for grid map visualization. Each (x,y) cell has only 1 corresponding depth value.
void NormalEstimator::set2dGridMap() {
    for (int x_idx=0;x_idx<GRID_CELL_WIDTH;x_idx++) {
        for (int y_idx=0;y_idx<GRID_CELL_HEIGHT;y_idx++) {
            depth_map[y_idx][x_idx] = *std::max_element(intermediate_depth_map[y_idx][x_idx].begin(),
                                                            intermediate_depth_map[y_idx][x_idx].end());
        }
    }
}

void NormalEstimator::publishPclFromDepthMap() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
    msg->header.frame_id = "camera_link";
    msg->height = 1;
    int32_t pcl_size = 0;

    // if true, publishes all points that are within the rectangular grid (multiple points per cell)
    // if false, only publishes a single depth value for each x,y cell (1 point per cell)
    bool publishAllDepthsAtCell = true; 


    for (int x_idx=0;x_idx<GRID_CELL_WIDTH;x_idx++) {
        for (int y_idx=0;y_idx<GRID_CELL_HEIGHT;y_idx++) {
            if (depth_map[y_idx][x_idx] > LOWEST_GRID_DEPTH) {
                double x = amapper_grid_map.fromXIndex(x_idx);
                double y = amapper_grid_map.fromYIndex(y_idx);
                if (publishAllDepthsAtCell) {
                    for (int z_idx=0;z_idx<intermediate_depth_map[y_idx][x_idx].size();z_idx++) {
                        if (intermediate_depth_map[y_idx][x_idx][z_idx] > LOWEST_GRID_DEPTH) {
                            msg->points.push_back (pcl::PointXYZ(x, y, intermediate_depth_map[y_idx][x_idx][z_idx]));
                            pcl_size++;
                        }
                    }
                }
                else {
                    msg->points.push_back (pcl::PointXYZ(x, y, depth_map[y_idx][x_idx]));
                    pcl_size++;
                }
            }
        }
    }
    msg->width = pcl_size;

    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pcl_pub.publish(msg);
    // ROS_INFO("Pointcloud Published. Size of PCL = %d", pcl_size);
}

void NormalEstimator::onPCLReceived(const pcl::PointCloud<pcl::PointXYZ> & pcl_msg) {
    std::vector<Eigen::Vector3d> points_arr;
    for (auto pcl_point : pcl_msg.points) {
        cv::Point3d pt(pcl_point.x, pcl_point.y, pcl_point.z);
        convertOpticalToEnuFrame(pt);
        if (amapper_grid_map.isWithinMetricMap(pt.x, pt.y)) {
            int x_idx = amapper_grid_map.toXIndex(pt.x);
            int y_idx = amapper_grid_map.toYIndex(pt.y);
            if (amapper_grid_map.isWithinGridCellMap(x_idx, y_idx)) {
                pt.z += GROUND_DEPTH_OFFSET;
                intermediate_depth_map[y_idx][x_idx].push_back(pt.z);
                int orig_val = amapper_grid_map.data[y_idx][x_idx];
                amapper_grid_map.data[y_idx][x_idx] = std::max(orig_val, int(100*pt.z)); // for debugging
                Eigen::Vector3d pos(pt.x, pt.y, pt.z);
                points_arr.push_back(pos);
            }
        }
    }

    set2dGridMap(); // convert intermediate depth map to Amapper Grid
    std::pair<Eigen::Vector3d, Eigen::Vector3d> plane_normal_properties = bestPlaneFromPoints(points_arr);
    // std::cout << "Center: " << plane_normal_properties.first << std::endl << "Slope: " << plane_normal_properties.second << std::endl;
    publishPlaneNormal(plane_normal_properties);
    publishPclFromDepthMap();

    nav_msgs::OccupancyGrid occupancy_grid_msg = amapper_grid_map.toOccupancyGrid();
    grid_pub.publish(occupancy_grid_msg);
    // ROS_INFO("Map Published");
    clearMapData();
}


int main(int argc,char** argv) {
    ros::init(argc, argv, "realsense_normal_estimates");
    NormalEstimator ne;
    ros::spin();
}