#include <data_compressor/parser.h>

int expectInt(PacketParser& parser, std::vector<uint8_t>& data) {
    int x  = 0;
    if(parser.index > data.size()-4)
        throw new ParseException("Expecting 4 bytes when parsing");
    x = data[parser.index] << 24;
    parser.index++;
    x |= data[parser.index] << 16;
    parser.index++;    
    x |= data[parser.index] << 8;
    parser.index++;
    x |= data[parser.index];
    parser.index++;
    return x;
}

uint16_t expectUInt16(PacketParser& parser, std::vector<uint8_t>& data) {
    uint16_t x  = 0;
    if(parser.index > data.size()-2)
        throw new ParseException("Expecting 4 bytes when parsing");
    x |= data[parser.index] << 8;
    parser.index++;
    x |= data[parser.index];
    parser.index++;
    return x;
}

int16_t expectInt16t(PacketParser& parser, std::vector<uint8_t>& data) {
    int16_t x  = 0;
    if(parser.index > data.size()-2)
        throw new ParseException("Expecting 4 bytes when parsing");
    x |= data[parser.index] << 8;
    parser.index++;
    x |= data[parser.index];
    parser.index++;
    return x;
}

int8_t expectInt8t(PacketParser& parser, std::vector<uint8_t>& data) {
    int8_t x  = 0;
    if(parser.index > data.size()-1)
        throw new ParseException("Expecting 4 bytes when parsing");
    x |= data[parser.index];
    parser.index++;
    return x;
}

uint8_t expectUInt8t(PacketParser& parser, std::vector<uint8_t>& data) {
    uint8_t x  = 0;
    if(parser.index > data.size()-1)
        throw new ParseException("Expecting 4 bytes when parsing");
    x |= data[parser.index];
    parser.index++;
    return x;
}

void encodeInt8t(std::vector<uint8_t>& packet, int8_t data){
    packet.push_back(data);
}

void encodeUInt8t(std::vector<uint8_t>& packet, uint8_t data){
    packet.push_back(data);
}

void encodeUInt16t(std::vector<uint8_t>& packet, uint16_t data){
    packet.push_back((data & 0xFF00) >> 8);
    packet.push_back(data);
}

void encodeInt16t(std::vector<uint8_t>& packet, int16_t data){
    packet.push_back((data & 0xFF00) >> 8);
    packet.push_back(data);
}
  
void encodeInt(std::vector<uint8_t>& packet, int data){
    packet.push_back((data & 0xFF000000) >> 24);
    packet.push_back((data & 0xFF0000) >> 16);
    packet.push_back((data & 0xFF00) >> 8);
    packet.push_back(data);
}