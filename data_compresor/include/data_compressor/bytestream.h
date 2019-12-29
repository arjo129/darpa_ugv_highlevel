#ifndef data_compression_bytestream_h
#define data_compression_bytestream_h

#include <stdint.h>
#include <vector>

struct ByteStream {
    std::vector<uint8_t> data;
};

#endif