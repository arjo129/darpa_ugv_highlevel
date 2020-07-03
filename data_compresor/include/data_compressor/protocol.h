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
    GOTO_GOAL = 0xF9,
    ESTOP_ACK = 0xF8,
    START = 0xF7,
    GOAL_ACK = 0xF6,
    DROP_NODE = 0xF5,
    AUTONOMOUS_NOW = 0xF4,
    AUTONOMOUS_ACK = 0xF3,
    TELEOP_NOW = 0xF2,
    TELEOP_ACK = 0xF1,
    DROP_NODE_ACK = 0xF0
};
#endif
