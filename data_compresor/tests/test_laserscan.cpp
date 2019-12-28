#include <data_compressor/devices/laserscan.h>
#include <gtest/gtest.h>
#include <random>
#include <limits>

TEST(LaserScan, is_compression_valid){
    data_compressor::LaserScanCompressor compressor;
    data_compressor::DataPacket packet;
    sensor_msgs::LaserScan scan;
    float inf = std::numeric_limits<float>::infinity();
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distr(0.1, 30);
    std::uniform_real_distribution<float> toInfOrNot(0, 1);
    for(int i = 0 ; i < 1000; i++) {
        if(toInfOrNot(generator) >0.99) {
            scan.ranges.push_back(inf);
        }
        else {
            scan.ranges.push_back(distr(generator));
        }
    }
    packet = compressor.compress(scan);
    sensor_msgs::LaserScan res = compressor.decompress(packet);
    for(int i = 0; i < res.ranges.size(); i++) {
        ASSERT_TRUE(res.ranges[i] == scan.ranges[i]);
    }
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
