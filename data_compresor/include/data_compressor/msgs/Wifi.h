#ifndef _DC_WIFI_H
#define _DC_WIFI_H
#include <string>

struct Wifi {
    std::string ssid;
    std::string signal;
    std::string quality;

    bool operator ==(const Wifi& other) const {
        if(this->ssid != other.ssid) {
            return false;
        }
        if(this->signal != other.signal) {
            return false;
        }
        if(this->quality != other.quality) {
            return false;
        }
        return true;
    }
};
#endif
