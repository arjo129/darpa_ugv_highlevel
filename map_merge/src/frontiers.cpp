#include <ros/ros.h>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <map_merge/laser_operations.h>
#include <nanoflann/nanoflann.hpp>
#include <tf/tf.h>

ros::Publisher pub;
pcl::PointXYZ scanPointToPointCloud(pcl::PointXYZ point, double azimuth); //Access private API
tf::TransformListener listener;

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

struct PositionStampedScan {
    LidarScan scan;
    Eigen::Matrix4f world_to_scan;
    
    bool isPointInsideScan(pcl::PointXYZ pt){
        Eigen::Vector4f point(pt.x, pt.y, pt.z, 1);
        Eigen::Vector4f tf = world_to_scan*point;
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

struct FrontierCloud {
    FrontierCloud_ frontiers;
    nanoflann::KDTreeSingleIndexDynamicAdaptor_<nanoflann::L2_Simple_Adaptor<float, FrontierCloud_>, FrontierCloud_, 3>* frontierAdaptor;

    FrontierCloud() {
        frontierAdaptor = new nanoflann::KDTreeSingleIndexDynamicAdaptor_<nanoflann::L2_Simple_Adaptor<float, FrontierCloud_>, FrontierCloud_, 3>(3, frontiers, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }

    void add(pcl::PointXYZ pt){
        //TODO: PErform better memory management
        auto index = frontiers.pts.size();
        frontiers.pts.push_back(pt);
        frontierAdaptor->addPoints(index, index);
    }

    void getNeighboursWithinRadius(pcl::PointXYZ pt, float radius, std::vector<int>& neighbours){
        float _pt[3];
        _pt[0] = pt.x;
        _pt[1] = pt.y;
        _pt[2] = pt.z;

        std::vector< std::pair<size_t, float>> indices;
        indices.reserve(1000);
        nanoflann::SearchParams params;
        frontierAdaptor->radiusSearch(_pt, radius, indices, params);

        for(auto r: indices) {
            neighbours.push_back(r);
        }
    }

    void removeIndex(int index) {
        frontierAdaptor->removePoint(index);
    }

    ~FrontierCloud(){
        delete frontierAdaptor;
    }
};

struct LidarCloud_ {
    std::vector<PositionStampedScan> pts;
    
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		return pts[ids].world_to_scan(3, dim);
	}

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

struct LidarHistory {
    LidarCloud_ scans;
    nanoflann::KDTreeSingleIndexDynamicAdaptor_<nanoflann::L2_Simple_Adaptor<float, FrontierCloud_>, FrontierCloud_, 3>* frontierAdaptor;

    FrontierCloud() {
        frontierAdaptor = new nanoflann::KDTreeSingleIndexDynamicAdaptor_<nanoflann::L2_Simple_Adaptor<float, FrontierCloud_>, FrontierCloud_, 3>(3, scans, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }

    void add(PositionStampedScan scan){
        //TODO: PErform better memory management
        auto index = scans.pts.size();
        scans.pts.push_back(scan);
        frontierAdaptor->addPoints(index, index);
    }

    void getNeighboursWithinRadius(pcl::PointXYZ pt, std::vector<int>& neighbours){
        float _pt[3];
        _pt[0] = pt.x;
        _pt[1] = pt.y;
        _pt[2] = pt.z;

        std::vector< std::pair<size_t, float>> indices;
        indices.reserve(1000);
        nanoflann::SearchParams params;
        frontierAdaptor->radiusSearch(_pt, 100, indices, params);

        for(auto r: indices) {
            neighbours.push_back(r);
        }
    }
    ~FrontierCloud(){
        delete frontierAdaptor;
    }
};


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
    for(int i = 0; i < 3; i++){
        trans(i,3) = v[i];
    }
    return trans;
}

struct FrontierStorage {
    
    LidarHistory scans;
    FrontierCloud frontiers; //TODO: Create some quadtree or smth... Currently this is a hack that will not hold up

    void addLidarScan(LidarScan& scan, tf::Transform position) {
        PositionStampedScan sc;
        sc.scan = scan;
        sc.world_to_scan = tfTransToEigen(position);
        auto v = position.getOrigin();
        pcl::PointXYZ pos;
        pos.x = v.x();
        pos.y = v.y();
        pos.z = v.z();
        std::vector<int> indices; 
        frontiers.getNeighboursWithinRadius(pos, sc.getMaxRadius(), indices);
        std::vector<int> to_be_removed;
        for(int index: indices) {
            if(isPointInside(scan, frontiers.frontiers.pts[index])) {
                //If the point is seen remove it;
                to_be_removed.push_back(index)
            }
        }
        for(auto r: to_be_removed) {
            frontiers.removeIndex(r);
        }
        scans.add(sc);
    }

    void addFrontiers(pcl::PointCloud<pcl::PointXYZ> points, Eigen::Matrix4f world_transform){
        pcl::PointCloud<pcl::PointXYZ> global_frame; 
        pcl::transformPointCloud(points, global_frame, world_transform);
        for(auto pt: global_frame) {
            //TODO check if we should add or not
            std::vector<int> lin;
            scans.getNeighbourWithRadius(pt, lin);
            bool isInside = false;
            for(auto l: lin){
                isInside |= isPointInside(scans.scans[l], pt);
            }
            if(!isInside) {
                frontiers.add(pt);
            }
        }
    }
};

void onPointCloudRecieved(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  pcl_msg) {
    
    tf::StampedTransform stamp;
    try{ 
        listener.lookupTransform(pcl_msg->header.frame_id, "world", ros::Time(pcl_msg->header.stamp), stamp);
    } catch (tf::TransformException tf) {
        ROS_WARN("Failed to transform tf");
        return;
    }

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
