#ifndef _data_compressor_h_
#define _data_compressor_h_

#include <data_compressor/datapacket.h>

namespace data_compressor {
    template<typename T>
    class Device {
    public:
        virtual std::string handlesDataPacket() = 0;
        virtual DataPacket compress(T message) = 0;
        virtual T decompress(DataPacket message) = 0;
    };
};
#endif