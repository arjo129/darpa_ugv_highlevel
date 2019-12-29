#ifndef _data_compressor_h_
#define _data_compressor_h_

#include <data_compressor/datapacket.h>

namespace data_compressor {
    template<typename T>
    class Device {
    public:
        virtual MessageType handlesDataPacket() = 0;
        virtual DataPacket compress(T message) = 0;
        virtual T decompress(DataPacket message) = 0;
    };

    enum class MessageType: uint16_t {
        LASER_SCAN, IMAGE, POINT_CLOUD
    };
};
#endif