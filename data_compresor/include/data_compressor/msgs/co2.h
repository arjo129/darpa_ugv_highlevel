#ifndef _DC_CO2_H
#define _DC_CO2_H
#include <stdint.h>
#include <vector>
#include <wireless_msgs/LoraPacket.h>
#include <wireless_msgs/Co2.h>

struct Co2 {
    uint64_t timestamp;
    int x,y,z;
    uint16_t concentration;

    bool operator ==(const Co2& other) const {
        if(this->timestamp != other.timestamp)
            return false;
        if(this->x != other.x)
            return false;
        if(this->y != other.y)
            return false;
        if(this->z != other.z)
            return false;
        if(this->concentration != other.concentration)
            return false;
        return true;
    }
};

wireless_msgs::Co2 toCo2(Co2 co2);
std::vector<uint8_t> encodeCo2(Co2 co2);
Co2 decodeCo2(std::vector<uint8_t> packet);
wireless_msgs::LoraPacket toLoraPacket(Co2 co2);
#endif
