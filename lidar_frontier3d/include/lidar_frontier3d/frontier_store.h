#ifndef __FRONTIER_STORE_H__
#define __FRONTIER_STORE_H__

#include <nanoflann/nanoflann.hpp>
#include <vector>
#include <queue>
#include <unordered_set>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <lidar_frontier3d/voxel_grid.h>
struct Frontier2D {
    Eigen::Vector3f start,end;

    Frontier2D(Eigen::Vector3f _start, Eigen::Vector3f _end) {
        start = _start;
        end = _end;
    }
    void toPointCloud(pcl::PointCloud<pcl::PointXYZ>& points, float resolution = 0.3) {
        auto diff = end-start;
        auto length = diff.norm();
        if(!std::isfinite(diff.x()) || !std::isfinite(diff.y()) || !std::isfinite(diff.z()))
            return;
        int max = length/resolution;
        /*for(int i = 0; i < max; i++) {
            auto res = resolution*i*diff/length;
            auto pt = start + res;
            points.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
        }*/
	auto pt = (start + end)/2.0;
	points.push_back(pcl::PointXYZ(pt.x() , pt.y() , pt.z()));
    }

    void getCentroids(pcl::PointCloud<pcl::PointXYZ>& points) {
        auto centroid = (start + end)/2;
        pcl::PointXYZ pt;
        pt.x = centroid.x();
        pt.y = centroid.y();
        pt.z = centroid.z();
        points.push_back(pt);
    }
};




struct FrontierCloud_ {
    std::vector<pcl::PointXYZ> pts;
    
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};



struct FrontierStore {
    FrontierCloud_ frontiers;
    nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<float, FrontierCloud_>, FrontierCloud_, 3>* frontierAdaptor;
    std::unordered_set<size_t> erased;
    FrontierStore() {
        frontierAdaptor = new nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<float, FrontierCloud_>, FrontierCloud_, 3>(3, frontiers, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }

    /**
     * Add a frontier: it must be in global frame
     */ 
    void add(pcl::PointXYZ pt){
        
        assert(std::isfinite(pt.x));

        if(!erased.empty()) {
            auto index = *erased.begin();
            frontiers.pts[index] = pt;
            frontierAdaptor->addPoints(index, index);
            erased.erase(index);
            return;
        }
        auto index = frontiers.pts.size();
        frontiers.pts.push_back(pt);
        frontierAdaptor->addPoints(index, index);
    }

    /**
     * Get neighbours within radius 
     */ 
    void getNeighboursWithinRadius(pcl::PointXYZ pt, std::vector<size_t>& neighbours, float max_dist=100, int initial_reserve=10000){
        float _pt[3];
        _pt[0] = pt.x;
        _pt[1] = pt.y;
        _pt[2] = pt.z;
        float* out_dist_sqr = (float*)malloc(initial_reserve*sizeof(float));
       
        size_t* indices = (size_t*)malloc(initial_reserve*sizeof(size_t));
        while(true) {
             for(int i = 0 ; i < initial_reserve; i++){
            out_dist_sqr[i] = INFINITY;
        }
            nanoflann::SearchParams params;
            nanoflann::KNNResultSet<float> resultSet(initial_reserve);
            resultSet.init(indices, out_dist_sqr);
            frontierAdaptor->findNeighbors(resultSet, _pt, nanoflann::SearchParams(10));
            int count = 0;
            for(int i = 0; i < initial_reserve; i++) {
                std::cout << out_dist_sqr[i] << std::endl; 
                if(out_dist_sqr[i] > max_dist*max_dist) goto cleanup;
                else count++;
            }
            if(count >= frontiers.kdtree_get_point_count()) {
                goto cleanup;
            }
            initial_reserve *= 2;
            out_dist_sqr = (float*)realloc(out_dist_sqr, initial_reserve*sizeof(float));
            indices = (size_t*)realloc(indices, initial_reserve*sizeof(size_t));
        }
cleanup:
        for(int i = 0; i < initial_reserve; i++){
            if(out_dist_sqr[i] < max_dist*max_dist) {
                neighbours.push_back(indices[i]);
            }
        }
        free(out_dist_sqr);
        free(indices);
    }

    void toPCLPoints(pcl::PointCloud<pcl::PointXYZ>& pointcloud){
        
        for(size_t i = 0; i < frontiers.pts.size(); i++) {
            if(std::isfinite(frontiers.pts[i].x)) {
                pointcloud.push_back(frontiers.pts[i]);
            }
        }
    }

    void removeIndex(size_t index) {
        frontierAdaptor->removePoint(index);
        erased.insert(index);
        frontiers.pts[index].x = INFINITY;
    }

    ~FrontierStore(){
        delete frontierAdaptor;
    }
};
#endif
