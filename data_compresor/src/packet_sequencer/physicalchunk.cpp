#include <data_compressor/datapacket.h>

using namespace data_compressor;

PhysicalChunk::PhysicalChunk(const PhysicalChunk& other) {
    this->maxCount = other.maxCount;
    this->sequenceNumber = other.sequenceNumber;
    this->packetId = other.packetId;
    for(int i = 0 ; i < other.data.size(); i++){
        this->data.push_back(other.data[i]);
    }
}

bool PhysicalChunk::operator< (const PhysicalChunk& other) const{
    return this->sequenceNumber < other.sequenceNumber; 
}

bool PhysicalChunk::operator== (const PhysicalChunk& other) const{
    if(this->sequenceNumber != other.sequenceNumber) {
        //std::cout << "wrong sequence number" <<std::endl;
        return false;
    }
    if(this->maxCount != other.maxCount) {
        //std::cout << "count" <<std::endl;
        return false;
    }
    if(this->packetId != other.packetId) {
        //std::cout << "wrong packet id" <<std::endl;
        return false;
    }
    if(this->data.size() != other.data.size()) {
        //std::cout << "data size mismatch" <<std::endl;
        return false;
    }
    for(int i = 0; i < this->data.size(); i++){
        if(this->data[i] != other.data[i]){
            //std::cout << "data invalid at index " << i <<std::endl;
            //std::cout << (int)this->data[i] << "," << (int)other.data[i] <<std::endl;
            return false;
        }
    }
    return true;
}

wireless_msgs::LoraPacket PhysicalChunk::toLoRa(std::string sender) {
    wireless_msgs::LoraPacket packet;
    packet.to.data = sender;
    union {
        uint64_t value;
        uint8_t bytes[8];
    } long_converter;
    long_converter.value = this->packetId;
    for(int i = 0 ; i < 8 ; i++)
        packet.data.push_back(long_converter.bytes[i]);
    
    union {
        int32_t value;
        uint8_t bytes[4];
    } int_converter;
    int_converter.value = this->sequenceNumber;
    for(int i = 0 ; i < 4 ; i++)
        packet.data.push_back(int_converter.bytes[i]);
    
    int_converter.value = this->maxCount;
    for(int i = 0 ; i < 4 ; i++)
        packet.data.push_back(int_converter.bytes[i]);

    for(int i = 0 ; i <  this->data.size(); i++)
        packet.data.push_back(this->data[i]);
    return packet;
}

PhysicalChunk data_compressor::toPhysicalChunk(wireless_msgs::LoraPacket& packet) {
    PhysicalChunk pchunk;
    int pointer = 0;
    
    union {
        uint64_t value;
        uint8_t bytes[8];
    } long_converter;
    
    for(; pointer < 8 ; pointer++) {
        long_converter.bytes[pointer] = packet.data[pointer];
    }
    pchunk.packetId = long_converter.value;

    union {
        int32_t value;
        uint8_t bytes[4];
    } int_converter;

    for(int i = 0; i < 4 ; i++)
        int_converter.bytes[i] = packet.data[pointer+i];
    pointer += 4;
    pchunk.sequenceNumber = int_converter.value;

    for(int i = 0; i < 4 ; i++)
        int_converter.bytes[i] = packet.data[pointer+i];
    pointer += 4;
    pchunk.maxCount = int_converter.value;
    
    for(; pointer < packet.data.size(); pointer++){
        pchunk.data.push_back(packet.data[pointer]);
    }
    return pchunk;
}

