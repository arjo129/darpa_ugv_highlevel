#ifndef _data_compressor_common_h_
#define _data_compressor_common_h_

#include <string>
#include <vector>
#include <queue>
#include <set>
#include <unordered_map>
#include <wireless_msgs/LoraPacket.h>
#include <geometry_msgs/Pose.h>

namespace data_compressor{

    const long MAX_PACKET_SIZE = 1000;
    const int MAX_TIME_TO_HOLD_PACKET = 20;

    struct ByteStream {
         std::vector<uint8_t> data;
    };
    /**
     * Compressed messages. Consist of data and a pose estimate.
     */ 
    struct DataPacket {
        geometry_msgs::Pose estimated_pose;
        std::string packet_type;
        std::vector<uint8_t> data;
        long uncompressed_size;
        ByteStream serialize();
    };

    DataPacket fromByteStream(ByteStream);

    /**
     * If messages need to be split up
     */ 
    struct PhysicalChunk {
        uint64_t packetId;
        int32_t sequenceNumber;
        int32_t maxCount;
        std::vector<uint8_t> data;
        PhysicalChunk() {};
        PhysicalChunk(const PhysicalChunk& other);
        bool operator< (const PhysicalChunk& other) const;
        bool operator== (const PhysicalChunk& other) const;
        /**
         * Convert to LoRA message
         */ 
        wireless_msgs::LoraPacket toLoRa(std::string sender);
    };
    /**
     * Create physical chunk from LoRa message
     */ 
    PhysicalChunk toPhysicalChunk(wireless_msgs::LoraPacket& packet);
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
    class PacketRecieveQueue {
    private:
        std::unordered_map<long, std::set<PhysicalChunk>> incomingPacketQueue;
        std::priority_queue<std::pair<long, long>> packetTimeofArrival;
    public:
        PacketRecieveQueue();
        /**
         * Gets all complete packages
         */ 
        std::vector<ByteStream> getBuffer();
        /**
         * Adds a chunk to the buffer
         */ 
        void enqueuePacket(PhysicalChunk packet, int arrivalTime);
        /**
         * Remove incomplete transmissions in the buffer which are of a certain age 
         */
        void garbageCollect(int currentTime); 
    };
    
    
    /**
     * Packet Queue send buffer
     */ 
    class PacketSendQueue {
    private:
        long packetCounter = 0;
        std::string localAddress;
    public:
        PacketSendQueue(std::string myaddress){
            localAddress = myaddress;
        }
        /**
         * Prepare a packet for sending
         */ 
        std::vector<wireless_msgs::LoraPacket> send(ByteStream bs, std::string to);
    };

    /**
     * Packet reciever
     */ 
}
#endif