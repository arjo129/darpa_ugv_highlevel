#include <ros/ros.h>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <map_merge/laser_operations.h>
#include <nanoflann/nanoflann.hpp>
#include <lidar_frontier3d/frontier_manager.h>
#include <lidar_frontier3d/frontier_request.h>
#include <tf/tf.h>

ros::Publisher visualization_pub, centroid_pub, local_pub;
pcl::PointXYZ scanPointToPointCloud(pcl::PointXYZ point, double azimuth); //Access private API
tf::TransformListener* listener;
FrontierManager manager;
FrontierStore seenPoints;
pcl::PointCloud<pcl::PointXYZ> pcloud;

void downsampleScan(const LidarScan& scan, LidarScan& out, int num) {
    for(auto ring: scan) {
        LidarRing new_ring;
        new_ring.azimuth = ring.azimuth;
        downsample(ring.scan, new_ring.scan, num);
        if(new_ring.scan.ranges.size() == 0) continue;
        out.push_back(new_ring);
    }
}

bool getFrontiers(lidar_frontier3d::frontier_request::Request &req,
                lidar_frontier3d::frontier_request::Response &res) {
    
    pcl::PointCloud<pcl::PointXYZ> cleaned;
    cleaned.header = pcloud.header;
    for(auto point: pcloud) {
        std::vector<size_t> neighbours;
        seenPoints.getNeighboursWithinRadius(point, neighbours, 2);
        if(neighbours.size() == 0) {
            cleaned.push_back(point);
        }
        seenPoints.add(point);
    }
    sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg(cleaned, pc);

    res.points = pc;
    return true;
}

void onPointCloudRecieved(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  pcl_msg) {
    
    tf::StampedTransform current_pose;
    try{
        auto time_now = ros::Time::now();
        listener->waitForTransform(pcl_msg->header.frame_id, "world", time_now, ros::Duration(1.0));
        listener->lookupTransform(pcl_msg->header.frame_id, "world", time_now/*pcl_conversions::fromPCL(pcl_msg->header.stamp)*/, current_pose);
    } catch (tf::TransformException error) {
        ROS_WARN("Failed to transform tf %s", error.what());
        return;
    }

    static LidarScan _lidar_scan;
    LidarScan lidar_scan;
    decomposeLidarScanIntoPlanes(*pcl_msg, lidar_scan);

    //downsampleScan(_lidar_scan, lidar_scan, 10);

    std::vector<Frontier2D> frontiers;
    for(auto& ring: lidar_scan) {
        pcl::PointXYZ prev_pt;
        
        bool first = true;
        for(int i = 0; i < ring.scan.ranges.size(); i++) {
            auto range = ring.scan.ranges[i];
            auto angle = i*ring.scan.angle_increment + ring.scan.angle_min;
            if(range > ring.scan.range_max || range < std::max(ring.scan.range_min, 0.7f)) continue;
            auto x = range*cos(angle);
            auto y = range*sin(angle);
            pcl::PointXYZ _pt(x,y,0);
            auto pt = scanPointToPointCloud(_pt, ring.azimuth);
            if(first) {
                first = false;
                prev_pt = _pt;
            }

            Eigen::Vector3f p1(prev_pt.x, prev_pt.y, prev_pt.z);
            Eigen::Vector3f p2(pt.x, pt.y, pt.z);

            auto length = (p2 - p1).norm();
            if(length > 1.5 && pt.z < 1) { //Onlu frontiers 1.5m and height less than 2m
                frontiers.push_back(Frontier2D(p1, p2));
            }
            prev_pt = pt;
        }
    }

    pcl::PointCloud<pcl::PointXYZ> local_frontiers;
    pcl::PointCloud<pcl::PointXYZ> centroid_points;
    local_frontiers.header = pcl_msg->header;
    centroid_points.header = pcl_msg->header;
    for(auto frontier: frontiers) {
        frontier.toPointCloud(local_frontiers);
        frontier.getCentroids(centroid_points);
    }


    manager.addLidarScan(lidar_scan, current_pose);

    pcl::PointCloud<pcl::PointXYZ> global_frame;
    pcl_ros::transformPointCloud(centroid_points, global_frame, current_pose.inverse());
    global_frame.header.frame_id  = "world";
    local_pub.publish(global_frame);
    
    pcl::PointCloud<pcl::PointXYZ> filtered;
    filtered.header = pcl_msg->header;
    filtered.header.frame_id  = "world";
    for(auto pt: global_frame) {
        if(!manager.queryFrontierPoint(pt) && pt.z < 2) {
            filtered.push_back(pt);
        }
    }
    centroid_pub.publish(filtered);
    pcloud = filtered;
    visualization_pub.publish(centroid_points);
}

int main(int argc, char** argv) {
    ros::init(argc, argv,"map_merge");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/X1/points/", 1, onPointCloudRecieved);
    ros::ServiceServer server = nh.advertiseService("/get_frontiers", getFrontiers);
   
    visualization_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/frontiers", 10);
    centroid_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/frontiers/centroid", 10);
    local_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/frontiers/local", 10);
    listener = new tf::TransformListener();
   
    int i = 0;
    while(ros::ok()) {
        ros::spinOnce();
        //if(i%10 == 0){
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.header.frame_id ="world";
            //cloud.header.stamp = ros::Time::now();
            manager.getFrontiers(cloud);
            //std::cout << "count"<< cloud.size() <<std::endl;
            pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
            cloud_filtered.header = cloud.header;
            for(auto p: cloud) {
                if(p.z< 2){
                    cloud_filtered.push_back(p);
                }
            }
            //pub.publish(cloud_filtered);
        //}
        i++;

    }
    return 0;
}
