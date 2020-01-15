#ifndef _PROTOCOL_H
#define _PROTOCOL_H
#include <stdint.h>
enum class MessageType: uint8_t {
    LASER_SCAN = 0xFF,
    WIFI_SIGNAL = 0xFE,
    HEAT_SIGNATURE_FRONT = 0xFD,
    HEAT_SIGNATURE_TOP = 0xFC,
    CO2_SIGNATURE = 0xFB, 
    ESTOP = 0xFA,
    GOTO_GOAL = 0xF9
};
#endif