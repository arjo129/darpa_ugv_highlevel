#include <ros/ros.h>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map_merge/laser_operations.h>
#include <map_merge/FALCOExtractor.h>


ros::Publisher pub, pub2;

void onPointCloudRecieved(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  pcl_msg) {

    std::unordered_map<long, pcl::PointCloud<pcl::PointXYZ> > planar_scans;
    /**
     * Decompose into scan planes
     */ 
    for(auto pt: *pcl_msg){
        if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        float r = sqrt(pt.x*pt.x + pt.y*pt.y);
        float angle = atan2(r,pt.z);
        float res  = angle/(2*M_PI)*360;
        long plane_hash = res;
        planar_scans[plane_hash].push_back(pt);
        planar_scans[plane_hash].header = pcl_msg->header;
    }

    FALKOExtractor falko;
    pcl::PointCloud<pcl::PointXYZ> cloud;
   /* for(auto plane: planar_scans) {
       std::vector<FALKO> corners;
       auto psc = toLaserScan(plane.second);
       //std::cout << __LINE__ <<" - " << psc.ranges.size();
       falko.extract(psc, plane.first*(2*M_PI)/360, corners);
       for(auto corner: corners) {
           pcl::PointXYZ pt(corner.point.x(), corner.point.y(), corner.point.z());
           cloud.push_back(pt);
       }
    }*/

    cloud.header = pcl_msg->header;
    auto scan = toLaserScan(planar_scans.begin()->second);
    naiveCornerDetector(scan, cloud, 10);
    pub.publish(cloud);
    pub2.publish(scan);
}

int main(int argc, char** argv) {
    ros::init(argc, argv,"map_merge");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/X1/points/", 10, onPointCloudRecieved);
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/corners", 10);

    pub2 = nh.advertise<sensor_msgs::LaserScan>("scan_las", 10);
    ros::spin();
    return 0;
}