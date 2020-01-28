#include <data_compressor/msgs/co2.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>

wireless_msgs::Co2 toCo2(Co2 co2) {
    wireless_msgs::Co2 msg;
    msg.header.frame_id = "co2";
    msg.header.stamp = ros::Time(co2.timestamp/1000000000ull, co2.timestamp);
    msg.position.x = co2.x;
    msg.position.y = co2.y;
    msg.position.z = co2.z;
    msg.concentration = co2.concentration;
    return msg;
}

std::vector<uint8_t> encodeCo2(Co2 co2) {
    std::vector<uint8_t>  bytestream;
    encodeUInt8t(bytestream, (uint8_t)MessageType::CO2_SIGNATURE); //1
    encodeUInt64t(bytestream, co2.timestamp); //9
    encodeInt(bytestream, co2.x); //12
    encodeInt(bytestream, co2.y); //16
    encodeInt(bytestream, co2.z); //20
    encodeUInt16t(bytestream, co2.concentration); //22
    return bytestream;
}

Co2 decodeCo2(std::vector<uint8_t> packet) {
    PacketParser state;
    Co2 co2;
    uint8_t type = expectInt8t(state, packet);
    co2.timestamp = expectUInt64t(state, packet);
    co2.x = expectInt(state, packet);
    co2.y = expectInt(state, packet);
    co2.z = expectInt(state, packet);
    co2.concentration = expectInt16t(state, packet);
    return co2;
}

wireless_msgs::LoraPacket toLoraPacket(Co2 co2) {
    wireless_msgs::LoraPacket packet;
    packet.to.data = "base_station";
    packet.data = compressZip(encodeCo2(co2));
}
