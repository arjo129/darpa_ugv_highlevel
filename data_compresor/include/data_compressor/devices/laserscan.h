#ifndef _data_compressor_lscan_h_
#define _data_compressor_lscan_h_
#include <data_compressor/data_compressor.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <zlib.h>

namespace data_compressor {
    class LaserScanCompressor: public Device<sensor_msgs::LaserScan> {
    public:
        LaserScanCompressor();
        MessageType handlesDataPacket();
        DataPacket compress(sensor_msgs::LaserScan message);
        sensor_msgs::LaserScan decompress(DataPacket message);
    };
}

#endif