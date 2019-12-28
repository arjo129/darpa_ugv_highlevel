#ifndef _data_compressor_common_h_
#define _data_compressor_common_h_
#include <string>
#include <vector>
namespace data_compressor{
    struct DataPacket {
        std::string packet_type;
        std::vector<uint8_t> data;
        long uncompressed_size;
    };
}
#endif