#ifndef _packets_h
#define _packets_h_

#include <vector>
#include <wireless_msgs/LoraPacket.h>
#include <teensy_bridge/NameRecord.h>
#include <memory>
class WirelessMessageHandler {
    std::shared_ptr<NameRecords> name;
public:
    WirelessMessageHandler(std::shared_ptr<NameRecords> _name){
        name = _name;
    }

    int serializeMessage(wireless_msgs::LoraPacket packet, uint8_t* buffer){
        buffer[0] = 0xFA;
        buffer[1] = name->getId(packet.to.data);
        int length = packet.data.size();
        buffer[2] = length >> 8;
        buffer[3] = length & 0xFF;
        for(int i = 0 ; i < length; i++) {
            buffer[i+4] = packet.data[i];
        }
        return length+4;
    }
};

enum class ParserState {
    AWAITING_START,
    DETERMINED_PACKETTYPE
};

enum class SerialResponseMessageType: uint8_t {
    PACKET_RECIEVED = 0xFA,
    PACKET_SENT = 0xFB,
    LORA_STATUS_READY = 0xFC,
    LORA_STATUS_BUSY = 0xFD,
    CO2_SENSOR_READING = 0xFE,
    THERMAL_FRONT = 0xFF,
    THERMAL_TOP = 0xF1,
    DEBUG = 0xF2
};

class SerialParser {
    ParserState state;
    SerialResponseMessageType messageType;
    std::shared_ptr<NameRecords> name;
    std::vector<uint8_t> packet;
    int packetLength;
    public:
    SerialParser(std::shared_ptr<NameRecords> _name) {
        this->name = _name;
        this->state = ParserState::AWAITING_START; 
    };

    /**
     * Add a byte as it comes in to the parser. 
     * @returns true if the packet is complete. False otherwise
     */ 
    bool addByteToPacket(uint8_t byte){
        if(byte!=252) std::cout << (int) byte << std::endl;
        if(this->state == ParserState::AWAITING_START && byte == (uint8_t)SerialResponseMessageType::PACKET_RECIEVED){ //Starting state. Check for LoRA
            this->state = ParserState::DETERMINED_PACKETTYPE;
            this->messageType = SerialResponseMessageType::PACKET_RECIEVED;
                    packet.push_back(byte);

            return false;
        }
        if(this->state == ParserState::AWAITING_START && byte == (uint8_t)SerialResponseMessageType::CO2_SENSOR_READING){ //Starting state. Check for LoRA
            this->state = ParserState::DETERMINED_PACKETTYPE;
            this->messageType = SerialResponseMessageType::CO2_SENSOR_READING;
                    packet.push_back(byte);

            return false;
        }
        if(this->state == ParserState::AWAITING_START && byte == (uint8_t)SerialResponseMessageType::LORA_STATUS_READY) {
            this->messageType = SerialResponseMessageType::LORA_STATUS_READY;
            return true;
        }
        if(this->state == ParserState::AWAITING_START)
            return false;
        packet.push_back(byte);

        if(this->messageType == SerialResponseMessageType::PACKET_RECIEVED) { //Parsing for LoRA packets
            if(packet.size() == 4) {
                this->packetLength = packet[2];
                this->packetLength <<= 8;
                this->packetLength = packet[3];  
            }
            if(packet.size() > 4 && packet.size() == this->packetLength+5)
                return true; //Return true if packet is complete
            return false;   
        }     
        if(this->messageType == SerialResponseMessageType::CO2_SENSOR_READING) {
            if(packet.size() == 12) {
                union{
                uint16_t value;
                uint8_t halves[2];
                } lower;
                lower.halves[0] =  packet[2];
                lower.halves[1] = packet[3];
                int concentration = lower.value;
                float humidity;
                memcpy(&humidity, packet.data()+4,4);
                float temp;
                memcpy(&temp, packet.data()+8,4);
                std::cout << "got CO2 packet" << concentration << ", " <<humidity  << ", " << temp << std::endl;
                std::cout <<std::endl;
                packet.clear();
                this->state = ParserState::AWAITING_START;
                return true;
            }
        }
        return false;
    }
    
    wireless_msgs::LoraPacket retrievePacket() {
        wireless_msgs::LoraPacket wpacket;
        wpacket.from.data = name->getName(this->packet[0]);
        wpacket.rssi = (int8_t)this->packet[this->packet.size()-1];
        for(int i =0; i < this->packetLength; i++){
            wpacket.data.push_back(this->packet[i + 4]);
        }
        this->state = ParserState::AWAITING_START; 
        this->packet.clear();
        return wpacket;
    }

    void reset() {
        this->state = ParserState::AWAITING_START; 
        this->packet.clear();
    }
    
    SerialResponseMessageType getMessageType() {
        return this->messageType;
    }
};
#endif