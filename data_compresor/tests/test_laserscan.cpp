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
    scan.timestamp = 1000l;
    scan.length = 100;
    scan.distances = new uint16_t[scan.length];
    for(int i  = 0; i < scan.length; i++) {
      scan.distances[i] = 24;
    }
    std::vector<uint8_t> data = encodeScan(scan);
    AdaptiveTelemetryScan result = decodeScan(data);
    ASSERT_EQ(scan, result);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
