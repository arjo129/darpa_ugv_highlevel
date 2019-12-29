#ifndef data_compression_bytestream_h
#define data_compression_bytestream_h

#include <stdint.h>
#include <vector>
namespace data_compressor {
struct ByteStream {
    std::vector<uint8_t> data;
};
}

#endif