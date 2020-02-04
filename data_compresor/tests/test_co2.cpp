#include <data_compressor/msgs/co2.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>
#include <gtest/gtest.h>
#include <random>
#include <limits>

TEST(Co2Compressor, compress_decompress_integration_test) {
    Co2 co2;
    co2.timestamp = 1000l;
    co2.x = 123;
    co2.y = 123;
    co2.z = 123;
    co2.concentration = 100;
    std::vector<uint8_t> data = encodeCo2(co2);
    Co2 result = decodeCo2(data);
    ASSERT_EQ(co2, result);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
