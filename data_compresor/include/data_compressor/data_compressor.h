#ifndef _data_compressor_h_
#define _data_compressor_h_

#include <data_compressor/datapacket.h>
#include <data_compressor/bytestream.h>

namespace data_compressor {

    enum class MessageType: uint16_t {
        LASER_SCAN, IMAGE, POINT_CLOUD, UWB, ESTOP
    };
    /**
     * Compressed messages. Consist of data and a pose estimate.
     */ 
    struct DataPacket {
        geometry_msgs::Pose estimated_pose;
        MessageType packet_type;
        long uncompressed_size;
        std::vector<uint8_t> data;
        DataPacket() {};
        DataPacket(const DataPacket& other);
        bool operator==(const DataPacket& other);
        /**
         * Serialize a ros datapacket to C
         */ 
        ByteStream serialize();
    };
    /**
     * Deserialize a ros datapacket
     */ 
    DataPacket fromByteStream(ByteStream bs);

    template<typename T>
    class Device {
    public:
        virtual MessageType handlesDataPacket() = 0;
        virtual DataPacket compress(T message) = 0;
        virtual T decompress(DataPacket message) = 0;
    };

    
};
#endif