#include <map_merge/laser_operations.h>
#include <gtest/gtest.h>

TEST(GAP_FILL, isCorrect) {
  sensor_msgs::LaserScan scan;
  scan.ranges.push_back(INFINITY);
  scan.ranges.push_back(10);
  scan.ranges.push_back(0);

  fillGaps(scan,1);

  ASSERT_EQ(scan.ranges[0], 5);
  ASSERT_EQ(scan.ranges[1], 10);
  ASSERT_EQ(scan.ranges[2], 0);
}

TEST(CENTROID_CORRECTNESS, isCorrect) {
  int N = 720;
  sensor_msgs::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI/N;
  scan.range_max = 20;

  auto curr_angle = scan.angle_min;
  for(int i = 0; i < N; i++) {
    scan.ranges.push_back(-cos(curr_angle)+sqrt(cos(curr_angle)*cos(curr_angle) + 10));
    scan.intensities.push_back(47);
  }

  sensor_msgs::LaserScan other;
  centroidNormalization(scan, other, 0.01);
  ASSERT_EQ(other.ranges.size(), scan.ranges.size());
  for(int i = 1; i < other.ranges.size(); i++) {
    ASSERT_LT(abs(other.ranges[i] - other.ranges[i-1]), 0.1) ;
    //std::cout << other.ranges[i] << std::endl;
  }
}

TEST(CENTROID_CORRECTNESS, handlesInfinityCorrectly) {
  int N = 720;
  sensor_msgs::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI/N;
  scan.range_max = 20;

  auto curr_angle = scan.angle_min;
  for(int i = 0; i < N; i++) {
    scan.ranges.push_back(INFINITY);
    scan.intensities.push_back(47);
  }

  sensor_msgs::LaserScan other;
  centroidNormalization(scan, other, 0.01);
  ASSERT_EQ(other.ranges.size(), scan.ranges.size());
  for(int i = 1; i < other.ranges.size(); i++) {
    ASSERT_EQ(other.ranges[i], INFINITY) ;
  }
}

TEST(DECOMPOSES_CORRECTLY, decomposeAndRecomposeCorrectly) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  auto TEST_SIZE = 360;
  for(int i = 0; i < TEST_SIZE; i++){
    auto yaw = i*2*M_PI/TEST_SIZE;
    pointCloud.push_back(pcl::PointXYZ(cos(yaw), sin(yaw), 2));
  }
  LidarScan scan;
  decomposeLidarScanIntoPlanes(pointCloud, scan);
  for(auto layer: scan) {
    for(int i = 0 ; i < layer.scan.ranges.size(); i++) {
      auto azimuth = layer.azimuth;
      auto yaw = layer.scan.angle_min + layer.scan.angle_increment*i;
      auto x = layer.scan.ranges[i] * cos(yaw);
      auto y = layer.scan.ranges[i] * sin(yaw); 
      if(!std::isfinite(x) || !std::isfinite(y)) continue;
      auto res = scanPointToPointCloud(pcl::PointXYZ(x,y,0), azimuth);
      ASSERT_NEAR(res.z, 2, 0.1);
    }
  }
}

TEST(IS_INSIDE, isCorrectlyDetectingInside) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  auto r = 10;
  for(float i = 0; i < 360; i++) {
    cloud.push_back(pcl::PointXYZ(r*cos(i/180*M_PI), r*sin(i/180*M_PI), 0));
    cloud.push_back(pcl::PointXYZ(r*cos(i/180*M_PI), r*sin(i/180*M_PI), 1));
    cloud.push_back(pcl::PointXYZ(r*cos(i/180*M_PI), r*sin(i/180*M_PI), -1));
  }
  LidarScan scan;
  decomposeLidarScanIntoPlanes(cloud, scan);

  pcl::PointXYZ squarelyInside(2, 2, 0.07);
  ASSERT_EQ(isPointInside(scan, squarelyInside), true);

  pcl::PointXYZ onlineInside(2, 2, 0);
  ASSERT_EQ(isPointInside(scan, onlineInside), true);

  pcl::PointXYZ outside(20, 20, 0);
  ASSERT_EQ(isPointInside(scan, outside), false);

  pcl::PointXYZ outside2(20, 20, 0.07);
  ASSERT_EQ(isPointInside(scan, outside2), false);

  pcl::PointXYZ outside3(2, 2, -100);
  ASSERT_EQ(isPointInside(scan, outside3), false);
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}