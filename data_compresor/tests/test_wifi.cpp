#include <data_compressor/msgs/WifiArray.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>
#include <gtest/gtest.h>
#include <random>
#include <limits>

TEST(WifiArrayCompressor, compress_decompress_integration_test) {
    WifiArray wa;
    wa.timestamp = 1000l;
    wa.x = 123;
    wa.y = 123;
    wa.z = 123;
    wa.length = 1;
    for (int i=0; i<wa.length; i++) {
        Wifi w;
        w.ssid = "nusseds";
        w.signal = "100";
        w.quality = "100";
        wa.data.push(w);
    }
    std::vector<uint8_t> data = encodeWifiArray(wa);
    WifiArray result = decodeWifiArray(data);
    ASSERT_EQ(wa, result);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
