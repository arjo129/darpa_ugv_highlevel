#include <ros/ros.h>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <map_merge/laser_operations.h>

ros::Publisher pub, pub2;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
pcl::PointCloud<pcl::PointXYZ> final_cloud1, reference;

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
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header = pcl_msg->header;
    extractCorners(*pcl_msg, cloud);
    final_cloud1 = cloud;
}

void onPointCloudRecieved2(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  pcl_msg) {

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

    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.header = pcl_msg->header;
    if(final_cloud1.size() != 0) {
        auto mat = ICPMatchPointToPoint(final_cloud1, *pcl_msg);
        std::cout << "Estimate" <<std::endl;
        std::cout << mat <<std::endl;
    }
    
    reference.clear();
    for(auto pt: *pcl_msg) 
    if(std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))reference.push_back(pt);
}

void visualize_thread() {
     pcl::PointCloud<pcl::PointXYZ>::Ptr final_points(new pcl::PointCloud<pcl::PointXYZ>);
    *final_points = final_cloud1;
    
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref(new pcl::PointCloud<pcl::PointXYZ>);
    *ref = reference; 

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1 (final_points, 0, 0, 200);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (ref, 0, 200, 0);

    
    std::cout <<ref->size() <<std::endl;
    viewer->removeAllPointClouds();
    viewer->addPointCloud(final_points, single_color1, "corners");
    viewer->addPointCloud(ref, single_color2, "reference");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "corners");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reference");
    viewer->resetCamera ();

}

int main(int argc, char** argv) {
    ros::init(argc, argv,"map_merge");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/X1/points/", 1, onPointCloudRecieved);
    ros::Subscriber sub2 = nh.subscribe("/X2/points/", 1, onPointCloudRecieved2);
    viewer->setBackgroundColor(0,0,0);
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/corners", 10);
    pub2 = nh.advertise<sensor_msgs::LaserScan>("scan_las", 10);
    while(ros::ok()) {
        ros::spinOnce();
        visualize_thread();
        viewer->spinOnce(100);
    }
    return 0;
}