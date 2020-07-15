#ifndef __FRONTIER_MANAGER_H__
#define __FRONTIER_MANAGER_H__

#include <lidar_frontier3d/frontier_store.h>
#include <lidar_frontier3d/lidar_history.h>
#include <lidar_frontier3d/voxel_grid.h>
#include <map_merge/laser_operations.h>
#include <tf/tf.h>

Eigen::Matrix4f tfTransToEigen(tf::Transform position){
    tf::Vector3 v= position.getOrigin();
    tf::Matrix3x3 m = position.getBasis();
    Eigen::Matrix4f trans;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j <3; j++) {
            trans(i,j) = m[i][j];
        }  
    }
    trans(3,0) = 0;
    trans(3,1) = 0;
    trans(3,2) = 0;
    trans(3,3) = 1;
    for(int i = 0; i < 3; i++) {
        trans(i,3) = v[i];
    }
    return trans;
}

struct FrontierManager {
    
    LidarHistory scans;
    FrontierStore frontiers;
    VoxelGrid<4> grid;

    void addLidarScan(LidarScan& scan, tf::Transform position) {
        PositionStampedScan sc;
        sc.scan = scan;
        sc.world_to_scan = tfTransToEigen(position);
        auto v = position.getOrigin();
        pcl::PointXYZ pos;
        pos.x = v.x();
        pos.y = v.y();
        pos.z = v.z();
        std::vector<size_t> indices; 
        for(auto ring: sc.scan) {
            for(int i = 0 ; i < ring.scan.ranges.size(); i++){
                auto point = scanPointToPointCloud(ring.scan, i, ring.azimuth);
                tf::Vector3 endTf(point.x, point.y, point.z);
                auto result = position*endTf;
                pcl::PointXYZ pt(result.x(), result.y(), result.z());
                grid.clearRay(pos, pt);
            }
        }
        scans.add(sc);
    }

    /**
     * Pass in global coordinates
     */ 
    void addFrontiers(pcl::PointCloud<pcl::PointXYZ>& points){
        /*long points_added = 0;
        for(auto pt: points) {
            //TODO check if we should add or not
            std::vector<size_t> neighbours;
            scans.getNeighboursWithinRadius(pt, neighbours);
            bool isInside = false;
            //std::cout << "Found scans:" << neighbours.size() <<std::endl;
            for(auto l: neighbours){
                isInside |= scans.scans.pts[l]->isPointInsideScan(pt);
            }
            if(!isInside) {
                frontiers.add(pt);
                grid.addPoint(pt);
                points_added++;
            }
        }
        std::cout << "Rejected "<< points.size() - points_added << ", added" << points_added << std::endl;*/
        for(auto pt: points) {
            if(!queryFrontierPoint(pt))
                grid.addPoint(pt);
        }
    }

    void getFrontiers(pcl::PointCloud<pcl::PointXYZ>& cloud) {
        //frontiers.toPCLPoints(cloud);
        grid.getPointCloud(cloud);
    }

    bool queryFrontierPoint(pcl::PointXYZ& pt) {
        std::vector<size_t> neighbours;
        scans.getNeighboursWithinRadius(pt, neighbours);
        //std::cout << "Found scans:" << neighbours.size() <<std::endl;
        for(auto l: neighbours){
            if(scans.scans.pts[l]->isPointInsideScan(pt)) return true;
        }
        return false;   
    }    
};

#endif