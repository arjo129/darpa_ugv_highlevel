#include <data_compressor/parser.h>
#include <iostream>

uint64_t expectUInt64t(PacketParser& parser, std::vector<uint8_t>& data) {
    uint64_t x  = 0;
    if(parser.index > data.size()-8)
        throw new ParseException("Expecting 8 bytes when parsing");
    x |= (uint64_t)data[parser.index] << 56;
    parser.index++;
    x |= (uint64_t)data[parser.index] << 48;
    parser.index++;    
    x |= (uint64_t)data[parser.index] << 40;
    parser.index++;
    x |= (uint64_t)data[parser.index] << 32;
    parser.index++; 
    x |= (uint64_t)data[parser.index] << 24;
    parser.index++;
    x |= (uint64_t)data[parser.index] << 16;
    parser.index++;    
    x |= (uint64_t)data[parser.index] << 8;
    parser.index++;
    x |= (uint64_t)data[parser.index];
    parser.index++;
    return x;
}

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

std::string expectString(PacketParser& parser, std::vector<uint8_t>& data) {
    /*
     * string = string length + string characters
     */
    uint8_t length = expectUInt8t(parser, data);
    if(parser.index > data.size()-length)
        throw new ParseException("Expecting more bytes than given length");
    std::string s = "";
    for (int i=0; i< length; i++) {
        s += data[parser.index];
        parser.index++;
    }
    return s;
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

void encodeUInt64t(std::vector<uint8_t>& packet, uint64_t data){
    packet.push_back((data & 0xFF00000000000000) >> 56);
    packet.push_back((data & 0xFF000000000000) >> 48);
    packet.push_back((data & 0xFF0000000000) >> 40);
    packet.push_back((data & 0xFF00000000) >> 32);
    packet.push_back((data & 0xFF000000) >> 24);
    packet.push_back((data & 0xFF0000) >> 16);
    packet.push_back((data & 0xFF00) >> 8);
    packet.push_back(data);
}

void encodeString(std::vector<uint8_t>& packet, std::string data){
    /*
     * string = string length + string characters
     */
    encodeUInt8t(packet, data.length());
    for (int i=0; i<data.length(); i++) {
        packet.push_back(data[i]);
    }
}
