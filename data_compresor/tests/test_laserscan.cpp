#include <data_compressor/msgs/laserscan.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>
#include <gtest/gtest.h>
#include <random>
#include <limits>

TEST(LaserScanCompressor, compress_decompress_integration_test) {
    
    AdaptiveTelemetryScan scan;
    scan.id = 0;
    scan.length = 100;
    scan.distances = new uint16_t[scan.length];
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
