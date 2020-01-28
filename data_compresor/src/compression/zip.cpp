#include <data_compressor/zip.h>
#include <zlib.h>

std::vector<uint8_t> compressZip(std::vector<uint8_t> packet){
    std::vector<uint8_t> finalPacket;
    unsigned long compressedLength = packet.size() + packet.size()/10 + 10;
    uint8_t* compressed = new uint8_t[compressedLength];
    compress2(compressed, &compressedLength, packet.data(), packet.size(), Z_BEST_COMPRESSION);
    for(int i = 0; i < compressedLength; i++){
        finalPacket.push_back(compressed[i]);
    }
    delete compressed;
    return finalPacket;
}

std::vector<uint8_t> uncompressZip(std::vector<uint8_t> packet){
    unsigned long size = packet.size()*5;
    uint8_t* decompressedByteStream = new uint8_t[size];
    int err = uncompress(decompressedByteStream, &size, packet.data(), packet.size());
    std::vector<uint8_t> result;
    if(err != Z_OK) {
        return result;
    }
    for(int i = 0; i < size; i++){
        result.push_back(decompressedByteStream[i]);
    }
    delete decompressedByteStream;
    return result;
}