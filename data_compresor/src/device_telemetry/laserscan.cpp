#include <data_compressor/devices/laserscan.h>

void compressScan(sensor_msgs::LaserScan sc) {
    
    std::stringstream compressed;
    boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
    out.push(boost::iostreams::zlib_compressor());

    for(int i =0; i < sc.ranges.size(); i++){
        char number[4];
        floatToBytes(sc.ranges[i], number);
        std::stringstream original;
        original << number[0] << number[1] <<number[2] << number[3];
        out.push(original);
    }
    
    wireless_msgs::LoraPacket loraPacket;
    loraPacket.data.push_back('l');
    loraPacket.data.push_back('s');

    boost::iostreams::copy(out, compressed);
    std::string buf = compressed.str();
    for(int i = 0; i < buf.size(); i++){
        loraPacket.data.push_back(buf[i]);
    }

}