#include <ros/ros.h>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <map_merge/laser_operations.h>

ros::Publisher pub, pub2;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
pcl::PointCloud<pcl::PointXYZ> final_cloud1, reference, transformed;

void removeFloorPoints(const pcl::PointCloud<pcl::PointXYZ>& pc1, pcl::PointCloud<pcl::PointXYZ>& out, double floor=-0.3){
    for(auto pt: pc1) {
        if(pt.z > floor)
            out.push_back(pt);
    }
}

void removeNans(const pcl::PointCloud<pcl::PointXYZ>& pc1, pcl::PointCloud<pcl::PointXYZ>& out){
    for(auto pt: pc1) {
        if(std::isfinite(pt.z) && std::isfinite(pt.x) && std::isfinite(pt.y))
            out.push_back(pt);
    }
}

void onPointCloudRecieved(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  pcl_msg) {

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header = pcl_msg->header;
    //extractCorners(*pcl_msg, final_cloud1);
    //removeNans(*pcl_msg, final_cloud1);
    final_cloud1 = *pcl_msg;
}

void onPointCloudRecieved2(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  pcl_msg) {

    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.header = pcl_msg->header;
    if(final_cloud1.size() != 0) {
        /*pcl::PointCloud<pcl::PointXYZ> my_no_floor, reference_no_floor;
        extractCorners(*pcl_msg, my_no_floor);
        auto mat = ICPMatchPointToPoint(final_cloud1, my_no_floor);
        std::cout << "Estimate" <<std::endl;
        std::cout << mat <<std::endl;
        pcl::transformPointCloud(final_cloud1, transformed, mat);*/
        LidarScan scanStack1, scanStack2;
        decomposeLidarScanIntoPlanes(final_cloud1, scanStack1);
        decomposeLidarScanIntoPlanes(*pcl_msg, scanStack2);
        std::vector<double> yaw_corrections;
        for(int i = 0; i < scanStack1.size(); i++) {
            if(i < 5) continue;
            EstimateYawCorrection(scanStack1[i].scan, scanStack2[i].scan, yaw_corrections);
        }

        double max_val = 0;
        int max_idx = 0;
        for(int i = 0; i < yaw_corrections.size(); i++) {
            if(yaw_corrections[i] > max_val){
                max_val = yaw_corrections[i];
                max_idx = i;
            }
        }

        auto med = max_idx*2*M_PI/181.0;
        Eigen::Transform<float, 3, Eigen::Affine> t;
        t = Eigen::Translation<float, 3>(0,0,0);
        t.rotate(Eigen::AngleAxis<float>(0, Eigen::Vector3f::UnitX()));
        t.rotate(Eigen::AngleAxis<float>(0, Eigen::Vector3f::UnitY()));
        t.rotate(Eigen::AngleAxis<float>(-med, Eigen::Vector3f::UnitZ()));
        
        pcl::PointCloud<pcl::PointXYZ> refined_corners, reference_corners, initial_guess;
        pcl::transformPointCloud(final_cloud1, initial_guess, t);
        extractCorners(initial_guess, refined_corners);
        extractCorners(*pcl_msg, reference_corners);
        auto transform = ICPMatchPointToPoint(refined_corners, reference_corners);
        pcl::transformPointCloud(initial_guess, transformed, transform);
    }
    
    reference.clear();
    for(auto pt: *pcl_msg) 
    if(std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))reference.push_back(pt);
}

void visualize_thread() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_points(new pcl::PointCloud<pcl::PointXYZ>);
    *final_points = transformed;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref(new pcl::PointCloud<pcl::PointXYZ>);
    *ref = reference; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr original(new pcl::PointCloud<pcl::PointXYZ>);
    removeNans(final_cloud1, *original); 

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1 (final_points, 0, 0, 200);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (ref, 0, 200, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3 (original, 200, 0, 0);
    
    viewer->removeAllPointClouds();
    viewer->addPointCloud(original, single_color3, "origin");
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