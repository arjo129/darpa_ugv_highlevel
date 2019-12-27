#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <wireless_msgs/LoraPacket.h>

void floatToBytes(float x, char* char_array) {
    memcpy(char_array, (char*)(&x), 4);
}

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "compression_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/husky1/scan", 10, &compressScan);   
}