#ifndef _DC_LSCAN_H
#define _DC_LSCAN_H
#include <stdint.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <wireless_msgs/LoraPacket.h>

struct AdaptiveTelemetryScan {
    uint64_t timestamp;
    uint16_t id;
    uint8_t sub_order;
    int x,y,z;
    int16_t rotateX, rotateY, rotateZ, rotateW;
    int length;
    uint16_t* distances;

    bool operator ==(const AdaptiveTelemetryScan& other) const {
        if(this->timestamp != other.timestamp)
            return false;
        if(this->id != other.id)
            return false;
        if(this->sub_order != other.sub_order)
            return false;
        if(this->x != other.x)
            return false;
        if(this->y != other.y)
            return false;
        if(this->z != other.z)
            return false;
        if(this->rotateX != other.rotateX)
            return false;
        if(this->rotateY != other.rotateY)
            return false;
        if(this->rotateZ != other.rotateZ)
            return false;
        if(this->rotateW != other.rotateW)
            return false;
        if(this->length != other.length)
            return false;
        for(int i = 0 ; i < this->length; i++){
            if(this->distances[i] != other.distances[i])
                return false;
        }
        return true;
    }
};

std::vector<AdaptiveTelemetryScan> convertLaserScan(sensor_msgs::LaserScan scan, nav_msgs::Odometry odom, int skip_vector);
sensor_msgs::LaserScan toLaserScan(AdaptiveTelemetryScan cscan);
std::vector<uint8_t> encodeScan(AdaptiveTelemetryScan scan);
AdaptiveTelemetryScan decodeScan(std::vector<uint8_t> packet);
wireless_msgs::LoraPacket toLoraPacket(AdaptiveTelemetryScan scan);
#endif