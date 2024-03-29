#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <nanoflann/nanoflann.hpp>
#include <vector>
#include <pcl/point_types.h>
#include <map_merge/laser_operations.h>

struct PositionStampedScan {
    LidarScan scan;
    Eigen::Matrix4f world_to_scan;
    
    bool isPointInsideScan(pcl::PointXYZ pt){
        Eigen::Vector4f point(pt.x, pt.y, pt.z, 1);
        Eigen::Vector4f tf = world_to_scan.inverse()*point;
        pcl::PointXYZ res(tf[0], tf[1], tf[2]);
        return isPointInside(scan, res);
    }

    float getMaxRadius(){
        float max = 0;
        for(auto& ring: scan) {
            max = std::max(ring.scan.range_max, max);
        }
        return max;
    }

    Eigen::Vector3f getPosition() {
        return Eigen::Vector3f(world_to_scan(3,0), world_to_scan(3,1), world_to_scan(3,2));
    }
};


struct LidarCloud_ {
    std::vector<PositionStampedScan*> pts;
    
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		return pts[idx]->world_to_scan(3, dim);
	}

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

    void clearAll() {
        for(auto p: pts) delete p;
    }
};

struct LidarHistory {
    LidarCloud_ scans;
    nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<float, LidarCloud_>, LidarCloud_, 3>* frontierAdaptor;

    LidarHistory() {
        frontierAdaptor = new nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<float, LidarCloud_>, LidarCloud_, 3>(3, scans, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }

    void add(PositionStampedScan& scan){
        //TODO: PErform better memory management
        auto index = scans.pts.size();
        PositionStampedScan* sc = new PositionStampedScan;
        sc->world_to_scan = scan.world_to_scan;
        for(auto ring: scan.scan){
            LidarRing new_ring;
            new_ring.azimuth = ring.azimuth;
            new_ring.scan.angle_increment = ring.scan.angle_increment;
            new_ring.scan.angle_max = ring.scan.angle_max;
            new_ring.scan.angle_min = ring.scan.angle_min;
            new_ring.scan.range_max = ring.scan.range_max;
            new_ring.scan.range_min = ring.scan.range_min;
            for(auto r: ring.scan.ranges) new_ring.scan.ranges.push_back(r);
            for(auto i: ring.scan.intensities) new_ring.scan.intensities.push_back(i);
            sc->scan.push_back(new_ring);
        }
        scans.pts.push_back(sc);
        frontierAdaptor->addPoints(index, index);
    }

    void getNeighboursWithinRadius(pcl::PointXYZ pt, std::vector<size_t>& neighbours, int max_results = 3000, float max_dist=100){
        float _pt[3];
        _pt[0] = pt.x;
        _pt[1] = pt.y;
        _pt[2] = pt.z;
        float out_dist_sqr[max_results];
        for(int i = 0 ; i < max_results; i++){
            out_dist_sqr[i] = INFINITY;
        }
        size_t indices[max_results];
        nanoflann::SearchParams params;
        nanoflann::KNNResultSet<float> resultSet(max_results);
        resultSet.init(indices, out_dist_sqr);
        frontierAdaptor->findNeighbors(resultSet, _pt, nanoflann::SearchParams(10));
        for(int i = 0; i < max_results; i++) {
            if(out_dist_sqr[i] < max_dist*max_dist)
                neighbours.push_back(indices[i]);
        }
    }
    ~LidarHistory(){
        delete frontierAdaptor;
        scans.clearAll();
    }
};
#endif