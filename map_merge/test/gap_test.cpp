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
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}