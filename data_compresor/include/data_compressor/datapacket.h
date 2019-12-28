#ifndef _data_compressor_common_h_
#define _data_compressor_common_h_

#include <string>
#include <vector>
#include <wireless_msgs/LoraPacket.h>
#include <queue>
#include <unordered_map>

namespace data_compressor{

    const long MAX_PACKET_SIZE = 1000;

    struct ByteStream {
        std::vector<uint8_t> data;
    };
    /**
     * Compressed messages
     */ 
    struct DataPacket {
        std::string packet_type;
        std::vector<uint8_t> data;
        long uncompressed_size;
    };

    /**
     * If messages need to be split up
     */ 
    struct PhysicalChunk {
        long packetId;
        int sequenceNumber;
        int maxCount;
        std::vector<uint8_t> data;
        bool operator< (const PhysicalChunk& other) const;
    };

    /**
     * Splits a packet into multiple physical chunks
     */ 
    std::vector<PhysicalChunk> toPhysicalChunks(ByteStream packet);

    /**
     * checks if a packet is ready to be reassembled
     */
    bool isPacketComplete(std::set<PhysicalChunk> fragments); 

    /**
     * Re-assembles a packet
     */ 
    ByteStream reassemblePacket(std::set<PhysicalChunk> fragments);

    /**
     * Packet Queue buffer
     */ 
    class PacketSequencer {
    private:
        std::unordered_map<long, std::set<PhysicalChunk>> incomingPacketQueue;
        std::priority_queue<std::pair<long, long>> packetTimeofArrival;
    public:
        PacketSequencer();
        void 
        
        fake();
        /**
         * Sends a buffer
         */ 
        //std::vector<wireless_msgs::LoraPacket> sendBuffer();
        /**
         * Gets a buffer
         */ 
        //std::vector<DataPacket> getBuffer();

    };
}
#endif