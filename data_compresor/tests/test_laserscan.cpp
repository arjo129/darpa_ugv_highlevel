#include <data_compressor/devices/laserscan.h>
#include <data_compressor/datapacket.h>
#include <gtest/gtest.h>
#include <random>
#include <limits>

TEST(LaserScan, isCmpressionValid){
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

TEST(PacketSequencer, isPacketCompleteCheckValid){
    using namespace data_compressor;
    ByteStream packet;
    std::default_random_engine generator;
    std::uniform_int_distribution<uint8_t> distr(0,255);
    
    for(int i = 0; i < 5*MAX_PACKET_SIZE; i++){
        packet.data.push_back(distr(generator));
    }

    std::vector<PhysicalChunk> chunks;
    chunks = toPhysicalChunks(packet);
    std::set<PhysicalChunk> test_queue;
    for(PhysicalChunk chunk: chunks){
        ASSERT_FALSE(isPacketComplete(test_queue));
        test_queue.insert(chunk);
    }
    ASSERT_TRUE(data_compressor::isPacketComplete(test_queue));
}

TEST(PacketSequencer, isPacketSizeBelowMaxChunk){
    using namespace data_compressor;
    ByteStream packet;
    std::default_random_engine generator;
    std::uniform_int_distribution<uint8_t> distr(0,255);
    
    for(int i = 0; i < 5*MAX_PACKET_SIZE; i++){
        packet.data.push_back(distr(generator));
    }

    std::vector<PhysicalChunk> chunks;
    chunks = toPhysicalChunks(packet);
    for(PhysicalChunk chunk: chunks){
        ASSERT_FALSE(chunk.data.size() > MAX_PACKET_SIZE);
    }
}

TEST(PacketSequencer, isSplittingAndReconstructionConsistent){
    using namespace data_compressor;
    ByteStream packet;
    std::default_random_engine generator;
    std::uniform_int_distribution<uint8_t> distr(0,255);
    
    for(int i = 0; i < 5*MAX_PACKET_SIZE; i++){
        packet.data.push_back(distr(generator));
    }

    std::vector<PhysicalChunk> chunks;
    chunks = toPhysicalChunks(packet);
    std::set<PhysicalChunk> test_queue;
    for(PhysicalChunk chunk: chunks){
        test_queue.insert(chunk);
    }

    ByteStream stream = reassemblePacket(test_queue);

    ASSERT_TRUE(stream.data.size() == packet.data.size());
    for(int i = 0; i < stream.data.size(); i++){
        ASSERT_TRUE(stream.data[i] == packet.data[i]);
    }
}

TEST(PhysicalChunk, equalityCheck) {
    std::default_random_engine generator;
    std::uniform_int_distribution<uint8_t> distr(0,255);
    data_compressor::PhysicalChunk pchunk1;
    pchunk1.maxCount = 2;
    pchunk1.packetId = 0;
    pchunk1.sequenceNumber =1;
    for(int i = 0; i < data_compressor::MAX_PACKET_SIZE; i++){
         pchunk1.data.push_back(distr(generator));
    }
    data_compressor::PhysicalChunk pchunk2(pchunk1);
    ASSERT_TRUE(pchunk1 == pchunk2);
    uint8_t tmp = pchunk1.data[0];
    pchunk1.data[0] = tmp-1;
    ASSERT_FALSE(pchunk1 == pchunk2);
}

TEST(PhysicalChunk, loraConversionValid) {
    std::default_random_engine generator;
    std::uniform_int_distribution<uint8_t> distr(0,255);
    data_compressor::PhysicalChunk pchunk1;
    pchunk1.maxCount = 2;
    pchunk1.packetId = 0;
    pchunk1.sequenceNumber =1;
    for(int i = 0; i < data_compressor::MAX_PACKET_SIZE; i++){
         pchunk1.data.push_back(distr(generator));
    }
    wireless_msgs::LoraPacket lorapacket = pchunk1.toLoRa("me");
    data_compressor::PhysicalChunk pchunk2;
    pchunk2 = data_compressor::toPhysicalChunk(lorapacket);
    ASSERT_TRUE(pchunk1 == pchunk2);
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
