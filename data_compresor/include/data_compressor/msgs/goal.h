#ifndef _GOAL_H_
#define _GOAL_H_
#include <wireless_msgs/LoraPacket.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>
#include <data_compressor/protocol.h>
struct Goal {
    int x,y; //Goal in *CM* with relation to the robot.

    bool operator ==(const Goal& other) const {
        if(this->x != other.x)
            return false;
        if(this->y != other.y)
            return false;
        return true;
    }
};
Goal decodeGoal(std::vector<uint8_t> packet) {
    PacketParser state;
    uint8_t type = expectUInt8t(state, packet);
    Goal goal;
    goal.x = expectUInt64t(state, packet);
    goal.y = expectUInt64t(state, packet);
    return goal;
}
wireless_msgs::LoraPacket toLoraPacket(Goal goal) {
    std::vector<uint8_t> data;
    encodeUInt8t(data, (uint8_t)MessageType::GOTO_GOAL);
    encodeUInt64t(data, goal.x);
    encodeUInt64t(data, goal.y);
    wireless_msgs::LoraPacket packet;
    packet.data = compressZip(data);
    return packet;
}
#endif