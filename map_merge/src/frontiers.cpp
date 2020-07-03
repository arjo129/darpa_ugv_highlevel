#include <ros/ros.h>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <map_merge/laser_operations.h>

ros::Publisher pub;
pcl::PointXYZ scanPointToPointCloud(pcl::PointXYZ point, double azimuth); //Access private API

struct Frontier2D {
    Eigen::Vector3f start,end;

    Frontier2D(Eigen::Vector3f _start, Eigen::Vector3f _end) {
        start = _start;
        end = _end;
    }
    void toPointCloud(pcl::PointCloud<pcl::PointXYZ>& points) {
        auto diff = end-start;
        auto length = diff.norm();
        if(!std::isfinite(diff.x()) || !std::isfinite(diff.y()) || !std::isfinite(diff.z()))
            return;
        int max = length/0.1;
        for(int i = 0; i < max; i++) {
            auto res = 0.1*i*diff/length;
            auto pt = start + res;
            points.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
        }
    }
};


struct Plane {
    Eigen::Vector3f normal, v1, v2, v3, v4;
    Plane(Eigen::Vector3f _v1, Eigen::Vector3f _v2, Eigen::Vector3f _v3, Eigen::Vector3f _v4){
        
        normal = (_v1-_v2).cross(_v3-_v2);
        v1 = _v1;
        v2 = _v2;
        v3 = _v3;
        v4 = _v4;
    }

    bool checkIfIntersects(Frontier2D& frontier, Eigen::Vector3f& v){
        auto slope = frontier.start - frontier.end;
        auto k = normal.dot(v4);
        auto x = k - normal.dot(frontier.end);
        auto m = slope.dot(normal);
        if(m == 0) return false;
        auto t = x/m;
        
        v = slope*t + frontier.end;
        return checkIfPointIsInside(v) && slope.dot(v-frontier.end) >= 0;
    }

    bool checkIfPointIsInside(Eigen::Vector3f v){
        auto x1 = (v-v1).dot(v2-v1);
        auto x2 = (v-v2).dot(v3-v2);
        auto x3 = (v-v3).dot(v4-v3);
        auto x4 = (v-v4).dot(v1-v4);
        return ((x1 > 0) && (x2 > 0) && (x3 > 0) && (x4 > 0)) || ((x1 < 0) && (x2 < 0) && (x3 < 0) && (x4 < 0));
    }
};

struct Frustum {
    Eigen::Vector3f left_bottom_back, left_bottom_front, left_top_back, left_top_front;
    Eigen::Vector3f right_bottom_back, right_bottom_front, right_top_back, right_top_front;

    bool checkIntersection(Frontier2D& frontier){
        
        Plane left(left_bottom_back, left_bottom_front, left_top_front, left_top_back);
        Plane right(right_bottom_back, right_bottom_front, right_top_front, right_top_back);
        Plane front(left_bottom_front, left_top_front, right_top_front, right_bottom_front);
        Plane back(left_bottom_back, left_top_back, right_top_back, right_bottom_back);
        Plane bottom(left_bottom_front, left_bottom_back, right_bottom_back, right_bottom_front);
        Plane top(left_top_front, left_top_back, right_top_back, right_top_front);

        Eigen::Vector3f rubbish;
        if(left.checkIfIntersects(frontier, rubbish)) return true;
        if(right.checkIfIntersects(frontier, rubbish)) return true;
        if(top.checkIfIntersects(frontier, rubbish)) return true;
        if(bottom.checkIfIntersects(frontier, rubbish)) return true;
        if(front.checkIfIntersects(frontier, rubbish)) return true;
        if(back.checkIfIntersects(frontier, rubbish)) return true;

        return false;
    }
};

void onPointCloudRecieved(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  pcl_msg) {
    
    LidarScan lidar_scan;
    decomposeLidarScanIntoPlanes(*pcl_msg, lidar_scan);

    std::vector<Frontier2D> frontiers;
    for(auto& ring: lidar_scan) {
        pcl::PointXYZ prev_pt;
        
        bool first = true;
        for(int i = 0; i < ring.scan.ranges.size(); i++) {
            auto range = ring.scan.ranges[i];
            auto angle = i*ring.scan.angle_increment + ring.scan.angle_min;
            if(range > ring.scan.range_max) continue;
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
            auto length = (p2 -p1).norm();
            if(length > 1) {
                frontiers.push_back(Frontier2D(p1, p2));
            }
            prev_pt = pt;
        }
    }

    pcl::PointCloud<pcl::PointXYZ> viz;
    viz.header = pcl_msg->header;
    for(auto frontier: frontiers) {
        frontier.toPointCloud(viz);
    }
    pub.publish(viz);
}

int main(int argc, char** argv) {
    ros::init(argc, argv,"map_merge");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/X1/points/", 1, onPointCloudRecieved);
   
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/frontiers", 10);

    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
