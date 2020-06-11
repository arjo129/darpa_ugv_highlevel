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

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}