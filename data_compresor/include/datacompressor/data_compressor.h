#ifndef _data_compressor_h_
#define _data_compressor_h_

#include <wireless_msgs/LoraPacket.h>

namespace data_compressor {
    template<typename T>
    class Device {
    public:
        virtual wireless_msgs::LoraPacket compress(T message) = 0;
        virtual wireless_msgs::LoraPacket decompress(T message) = 0;
    }
};
#endif