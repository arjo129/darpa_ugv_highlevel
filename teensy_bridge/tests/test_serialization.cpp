#include <gtest/gtest.h>
#include <random>
#include <limits>
#include <teensy_bridge/SerialPackets.h>

TEST(SerialPacket, conversionConsistent) {
    std::shared_ptr<NameRecords> record(new NameRecords);
    WirelessMessageHandler handler(record);
    SerialParser parser(record);
    record->addNameRecord("husky1", 1);
    wireless_msgs::LoraPacket packet;
    packet.to.data = "husky1";
    for(uint8_t i = 0; i <100; i++){
        packet.data.push_back(i);
    } 
    uint8_t buffer[255];
    //std::cout << "run_test" << __FILE__ << ":" <<__LINE__ <<std::endl;
    int length = handler.serializeMessage(packet, buffer);
    buffer[length] = 12;
    bool ready = false;
    for(int i = 0 ; i < length+1; i++){
        ready = parser.addByteToPacket(buffer[i]);
        if(i<length){
            ASSERT_FALSE(ready);
        }
    }
    ASSERT_TRUE(ready);
    wireless_msgs::LoraPacket result = parser.retrievePacket();
    ASSERT_EQ(result.data, packet.data);
    ASSERT_EQ(result.rssi, 12);
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}