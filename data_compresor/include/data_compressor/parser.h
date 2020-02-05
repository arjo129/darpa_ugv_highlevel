#ifndef _PARSER_H_
#define _PARSER_H_

#include <stdint.h>
#include <vector>
#include <string>

struct PacketParser{
    size_t index;

    PacketParser():index(0){}
};

struct ParseException : public std::exception {
    std::string msg;
    ParseException(std::string _msg): msg(_msg) {}
   const char * what () const throw () {
      return msg.c_str();
   }
};
uint64_t expectUInt64t(PacketParser& parser, std::vector<uint8_t>& data);
int expectInt(PacketParser& parser, std::vector<uint8_t>& data);
uint16_t expectUInt16(PacketParser& parser, std::vector<uint8_t>& data);
int16_t expectInt16t(PacketParser& parser, std::vector<uint8_t>& data);
int8_t expectInt8t(PacketParser& parser, std::vector<uint8_t>& data);
uint8_t expectUInt8t(PacketParser& parser, std::vector<uint8_t>& data);
std::string expectString(PacketParser& parser, std::vector<uint8_t>& data);

void encodeInt8t(std::vector<uint8_t>& packet, int8_t data);
void encodeUInt8t(std::vector<uint8_t>& packet, uint8_t data);
void encodeUInt16t(std::vector<uint8_t>& packet, uint16_t data);
void encodeInt16t(std::vector<uint8_t>& packet, int16_t data);
void encodeInt(std::vector<uint8_t>& packet, int data);
void encodeUInt64t(std::vector<uint8_t>& packet, uint64_t data);
void encodeString(std::vector<uint8_t>& packet, std::string data);
#endif
