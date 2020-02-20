#ifndef _DC_WIFIARR_H
#define _DC_WIFIARR_H
#include <stdint.h>
#include <vector>
#include <wireless_msgs/LoraPacket.h>
#include <wireless_msgs/WifiArray.h>
#include <data_compressor/msgs/Wifi.h>
#include <data_compressor/parser.h>

struct WifiArray {
    uint64_t timestamp;
    int x,y,z;
    uint8_t length;
    std::vector<Wifi> data;

    bool operator ==(const WifiArray& other) const {
        if(this->timestamp != other.timestamp)
            return false;
        if(this->x != other.x)
            return false;
        if(this->y != other.y)
            return false;
        if(this->z != other.z)
            return false;
        if(this->length != other.length)
            return false;
        for(int i = 0 ; i < this->length; i++){
            if(!(this->data[i] == other.data[i]))
                return false;
        }
        return true;
    }
};

wireless_msgs::WifiArray toWifiArray(WifiArray wa);
std::vector<uint8_t> encodeWifiArray(WifiArray wa);
void encodeWifi(std::vector<uint8_t>& bytestream, Wifi w);
WifiArray decodeWifiArray(std::vector<uint8_t> packet);
Wifi decodeWifi(PacketParser& state, std::vector<uint8_t>& packet);
wireless_msgs::LoraPacket toLoraPacket(WifiArray wa);
wireless_msgs::LoraPacket toLoraPacket(wireless_msgs::WifiArray array);
#endif
