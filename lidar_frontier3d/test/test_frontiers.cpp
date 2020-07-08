#include <lidar_frontier3d/lidar_history.h>
#include <lidar_frontier3d/frontier_store.h>
#include <lidar_frontier3d/frontier_manager.h>
#include <gtest/gtest.h>

Eigen::Matrix4f getPositionMatrix(float x, float y, float z) {
    Eigen::Matrix4f mat;
    mat.setZero();
    mat(3,0) = x;
    mat(3,1) = y;
    mat(3,2) = z;
    mat(3,3) = 1;
    mat(2,2) = 1;
    mat(1,1) = 1;
    mat(0,0) = 1;
    return mat;
}

void getLidar(LidarScan& scan, float radius) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    auto r = radius;
    for(float i = 0; i < 360; i++) {
        if(i > 100) {
        cloud.push_back(pcl::PointXYZ(r*cos(i/180*M_PI), r*sin(i/180*M_PI), 0));
        cloud.push_back(pcl::PointXYZ(r*cos(i/180*M_PI), r*sin(i/180*M_PI), 1));
        cloud.push_back(pcl::PointXYZ(r*cos(i/180*M_PI), r*sin(i/180*M_PI), -1));
        }
    }
}

tf::Transform getTransform(float x, float y, float z){
    tf::Transform t;
    t.setOrigin(tf::Vector3(x,y,z));
    t.setRotation(tf::Quaternion(0,0,0,1));
    return t;
}

TEST(FrontierStore, radiusSearch) {
    FrontierStore store;
    for(int i = 0; i < 100; i++){
        store.add(pcl::PointXYZ(i,0,0));
    }

    std::vector<size_t> neighbours;
    store.getNeighboursWithinRadius(pcl::PointXYZ(20,0,0), neighbours, 10, 10);

    ASSERT_EQ(neighbours.size(), 19);

    std::vector<size_t> neighbours2;
    store.getNeighboursWithinRadius(pcl::PointXYZ(20,0,0), neighbours2, 1000);
    ASSERT_EQ(neighbours2.size(), 100);
}

TEST(FrontierStore, deletionWorks) {
    FrontierStore store;
    for(int i = 0; i < 100; i++){
        store.add(pcl::PointXYZ(i,0,0));
    }

    std::vector<size_t> neighbours;
    store.getNeighboursWithinRadius(pcl::PointXYZ(20,0,0), neighbours, 10);

    ASSERT_EQ(neighbours.size(), 19);

    for(auto i: neighbours){
        store.removeIndex(i);
    }
    neighbours.clear();
    store.getNeighboursWithinRadius(pcl::PointXYZ(20,0,0), neighbours, 10);

    ASSERT_EQ(neighbours.size(), 0);
}

TEST(LidarHistory, basicOperations) {
    LidarHistory history;
    for(int i = 0; i < 10; i++){
        PositionStampedScan scan;
        scan.world_to_scan = getPositionMatrix(i*30, i*30, 0);
        history.add(scan);
    }
    std::vector<size_t> res;
    history.getNeighboursWithinRadius(pcl::PointXYZ(40,20,0), res, 30, 20);
    ASSERT_EQ(res.size(), 1);
    for(size_t index: res) {
        auto l = history.scans.pts[index].getPosition() - Eigen::Vector3f(40,20,0);
        ASSERT_LE(l.norm(), 20);
    }
}

TEST(FrontierManager, transformsCorrectly) {
    FrontierManager manager;
    
    LidarScan scan;
    getLidar(scan, 10);

    ASSERT_TRUE(isPointInside(scan, pcl::PointXYZ(0,0,0)));

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0,0,0));
    manager.addFrontiers(cloud);
    
    cloud.clear();
    manager.getFrontiers(cloud);
    ASSERT_EQ(cloud.size(), 1);

    cloud.clear();
    manager.addLidarScan(scan, getTransform(0,0,0));
    manager.getFrontiers(cloud);
    ASSERT_EQ(cloud.size(), 0);

    cloud.push_back(pcl::PointXYZ(0,0,0));
    manager.addFrontiers(cloud);
    cloud.clear();
    manager.getFrontiers(cloud);
    ASSERT_EQ(cloud.size(), 0);

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}