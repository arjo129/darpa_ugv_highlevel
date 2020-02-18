#include <data_compressor/msgs/laserscan.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>

std::vector<AdaptiveTelemetryScan> convertLaserScan(sensor_msgs::LaserScan scan, nav_msgs::Odometry odom, int skip_vector) {
    static uint16_t lastId;
    std::vector<AdaptiveTelemetryScan> scanFragments(skip_vector);
    for(int i = 0; i < skip_vector; i++) {
        scanFragments[i].timestamp = ros::Time::now().toNSec();
        scanFragments[i].id = lastId;
        scanFragments[i].sub_order = i;
        scanFragments[i].distances = new uint16_t[scan.ranges.size()/skip_vector];
        scanFragments[i].length = scan.ranges.size()/skip_vector;
        scanFragments[i].x = odom.pose.pose.position.x;
        scanFragments[i].y = odom.pose.pose.position.y;
        scanFragments[i].z = odom.pose.pose.position.z;
        scanFragments[i].rotateX = odom.pose.pose.orientation.x;
        scanFragments[i].rotateY = odom.pose.pose.orientation.y;
        scanFragments[i].rotateZ = odom.pose.pose.orientation.z;
        scanFragments[i].rotateW = odom.pose.pose.orientation.w;
    }
    for(int i = 0; i < scan.ranges.size(); i++) {
        if(isinf(scan.ranges[i]))
            scanFragments[i%skip_vector].distances[i/skip_vector] = 0;
        else
            scanFragments[i%skip_vector].distances[i/skip_vector] = (uint16_t)(scan.ranges[i]*1000.0f);
    }
    lastId++;
    return scanFragments;
}

sensor_msgs::LaserScan toLaserScan(AdaptiveTelemetryScan cscan) {
    sensor_msgs::LaserScan scan;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = (2*M_PI)/cscan.length;
    scan.range_max = 12;
    scan.range_min = 0.15;
    scan.header.frame_id = "laser";
    scan.header.stamp =ros::Time::now();
    for(int i = 0; i < cscan.length; i++) {
        if(cscan.distances[i] == 0){
            scan.ranges.push_back(INFINITY);
            scan.intensities.push_back(47.0);
            continue;
        }
        scan.ranges.push_back((float)cscan.distances[i]);
        scan.intensities.push_back(47.0);
        scan.ranges[scan.ranges.size()-1] /= 1000;
    }
    
    return scan;
}

nav_msgs::Odometry getOdom(AdaptiveTelemetryScan scan) {
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = scan.x;
    odom.pose.pose.position.y = scan.y;
    odom.pose.pose.position.z = scan.z;
    odom.pose.pose.orientation.w = scan.rotateW;
    odom.pose.pose.orientation.x = scan.rotateX;
    odom.pose.pose.orientation.y = scan.rotateY;
    odom.pose.pose.orientation.z = scan.rotateZ;
    return odom; 
}

std::vector<uint8_t> encodeScan(AdaptiveTelemetryScan scan) {
    std::vector<uint8_t>  bytestream;
    encodeUInt8t(bytestream, (uint8_t)MessageType::LASER_SCAN); //1
    encodeUInt64t(bytestream, scan.timestamp); //9
    encodeUInt16t(bytestream, scan.id); //11
    encodeUInt8t(bytestream, scan.sub_order); //12
    encodeInt(bytestream, scan.x); //16
    encodeInt(bytestream, scan.y); //20
    encodeInt(bytestream, scan.z); //24
    encodeInt16t(bytestream, scan.rotateX); //26
    encodeInt16t(bytestream, scan.rotateY); //28
    encodeInt16t(bytestream, scan.rotateZ); //30
    encodeInt16t(bytestream, scan.rotateW); //32
    for(int i = 0; i < scan.length; i++){
        encodeInt16t(bytestream, scan.distances[i]);
    }
    return bytestream;
}

AdaptiveTelemetryScan decodeScan(std::vector<uint8_t> packet) {
    PacketParser state;
    AdaptiveTelemetryScan scan;
    scan.length = (packet.size() - 32)/2;
    scan.distances = new uint16_t[scan.length];
    uint8_t type = expectInt8t(state, packet);
    scan.timestamp = expectUInt64t(state, packet);
    scan.id = expectInt16t(state, packet);
    scan.sub_order = expectInt8t(state, packet);
    scan.x = expectInt(state, packet);
    scan.y = expectInt(state, packet);
    scan.z = expectInt(state, packet);
    scan.rotateX = expectInt16t(state, packet);
    scan.rotateY = expectInt16t(state, packet);
    scan.rotateZ = expectInt16t(state, packet);
    scan.rotateW = expectInt16t(state, packet);
    for(int i = 0; i < scan.length; i++){
        scan.distances[i] = expectUInt16(state, packet);
    }
    return scan; 
}



wireless_msgs::LoraPacket toLoraPacket(AdaptiveTelemetryScan scan) {
    wireless_msgs::LoraPacket packet;
    packet.to.data = "base_station";
    packet.data = compressZip(encodeScan(scan));
    return packet;
}