#include <data_compressor/msgs/WifiArray.h>
#include <data_compressor/msgs/goal.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>
#include <gtest/gtest.h>
#include <random>
#include <limits>


TEST(GoalCompressor, compress_decompress_integration){
  Goal goal;
  goal.x = -123;
  goal.y = 56;
  wireless_msgs::LoraPacket packet = toLoraPacket(goal);
  std::vector<uint8_t> data = uncompressZip(packet.data);
  Goal res = decodeGoal(data);
  ASSERT_EQ(goal,res);

}

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
        wa.data.push_back(w);
    }

    std::vector<uint8_t> data = encodeWifiArray(wa);
    WifiArray result = decodeWifiArray(data);
    ASSERT_EQ(wa, result);
}

TEST(WifiArrayCompressor, compress_string_test) {
  std::string str = "hello";
  std::vector<uint8_t> packet;
  encodeString(packet, str);
  PacketParser parser;
  std::string result = expectString(parser, packet);
  ASSERT_EQ(result, str);
}

TEST(WifiArrayCompresor, compress_wifi) {
  Wifi w;
  w.ssid = "nusseds";
  w.signal = "100";
  w.quality = "100";
  std::vector<uint8_t> data;
  encodeWifi(data,w);
  PacketParser parser;
  Wifi res = decodeWifi(parser,data);
  ASSERT_EQ(res, w);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
