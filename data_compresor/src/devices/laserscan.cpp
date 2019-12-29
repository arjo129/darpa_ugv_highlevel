#include <data_compressor/devices/laserscan.h>

using namespace data_compressor;

void floatToBytes(float x, uint8_t* char_array) {
    memcpy(char_array, (uint8_t*)(&x), 4);
}

LaserScanCompressor::LaserScanCompressor() {}

MessageType LaserScanCompressor::handlesDataPacket(){
    return MessageType::LASER_SCAN;
}

DataPacket LaserScanCompressor::compress(sensor_msgs::LaserScan sc) {
    
    uint8_t* data = new uint8_t[sc.ranges.size()*4];
    uint8_t* compressed = new uint8_t[sc.ranges.size()*4];

    for(int i =0; i < sc.ranges.size(); i++){
        floatToBytes(sc.ranges[i], &(data[i*4]));
    }

    unsigned long compressedLength;
    compress2(compressed, &compressedLength, data, sc.ranges.size()*4, Z_BEST_COMPRESSION);
    delete data;
    
    DataPacket packet;
    
    for(unsigned long i = 0; i < compressedLength; i++){
        packet.data.push_back(compressed[i]);
    }

    packet.uncompressed_size = sc.ranges.size()*4;

    delete compressed;
    return packet;
}

sensor_msgs::LaserScan LaserScanCompressor::decompress(DataPacket dp) {
    
    uint8_t* compressedData = new uint8_t[dp.data.size()];
    uint8_t* decompressedByteStream = new uint8_t[dp.uncompressed_size];

    for(unsigned long i = 0; i < dp.data.size(); i++){
        compressedData[i] = dp.data[i];
    }
    
    unsigned long sz = dp.uncompressed_size;
    uncompress(decompressedByteStream, &sz, compressedData, dp.data.size());
    delete compressedData;

    sensor_msgs::LaserScan scan;
    for(unsigned long i = 0; i < sz; i+=4) {
        union {
            float float_value;
            char data[4];
        } fconverter;
        fconverter.data[0] = decompressedByteStream[i];
        fconverter.data[1] = decompressedByteStream[i+1];
        fconverter.data[2] = decompressedByteStream[i+2];
        fconverter.data[3] = decompressedByteStream[i+3];
        scan.ranges.push_back(fconverter.float_value);
    }
    delete decompressedByteStream;
    return scan;
}