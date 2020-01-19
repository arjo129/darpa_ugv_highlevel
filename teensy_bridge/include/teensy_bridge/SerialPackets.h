#ifndef _packets_h
#define _packets_h_

#include <vector>
#include <wireless_msgs/LoraPacket.h>
#include <teensy_bridge/NameRecord.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define THRESHOLD 30
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
        if(this->state == ParserState::AWAITING_START && byte == (uint8_t)SerialResponseMessageType::THERMAL_FRONT){ //Starting state. Check for LoRA
            this->state = ParserState::DETERMINED_PACKETTYPE;
            this->messageType = SerialResponseMessageType::THERMAL_FRONT;
                    packet.push_back(byte);

            return false;
        }
        if (this->state == ParserState::AWAITING_START) {
            return false;
        }
packet.push_back(byte);
        if(this->messageType == SerialResponseMessageType::PACKET_RECIEVED) { //Parsing for LoRA packets
            if(packet.size() == 3) {
                this->packetLength = packet[1];
                this->packetLength <<= 8;
                this->packetLength = packet[2];  
            }
            if(packet.size() > 3 && packet.size() == this->packetLength+4)
                return true; //Return true if packet is complete
            return false;   
        }     
        if(this->messageType == SerialResponseMessageType::CO2_SENSOR_READING) {
            if(packet.size() == 12) {

            }
        }
        if(this->messageType == SerialResponseMessageType::THERMAL_FRONT) {
            if(packet.size() == 779) {
                return true;
            }
            return false;
        }   
        return false;
    }
    wireless_msgs::LoraPacket retrievePacket() {
        wireless_msgs::LoraPacket wpacket;
        wpacket.from.data = name->getName(this->packet[0]);
        wpacket.rssi = this->packet[this->packet.size()-1];
        for(int i =0; i < this->packetLength; i++){
            wpacket.data.push_back(this->packet[i + 3]);
        }
        this->state = ParserState::AWAITING_START; 
        this->packet.clear();
        return wpacket;
    }

    cv::Mat retrieveThermalPacket() {
        //check that they are all thermal packets
        std::cout << "Processing thermal packets" << std::endl;
        this->packet.erase(this->packet.begin() + 585, this->packet.begin() + 588);
        this->packet.erase(this->packet.begin() + 390, this->packet.begin() + 393);
        this->packet.erase(this->packet.begin() + 195, this->packet.begin() + 198);
        this->packet.erase(this->packet.begin(), this->packet.begin() + 3);
        uint8_t* res = &this->packet[0];
        cv::Mat img(24, 32, CV_8UC1, cv::Scalar(0));
        for (int i = 0; i < 24; i++) {
            std::cout << std::endl;
            for (int j = 0; j < 32; j++) {
                std::cout << (int)res[i*32 + j] << "|";
                    img.at<uchar>(i,j) = (int)res[i*32 + j];
            }
        }
        this->state = ParserState::AWAITING_START;
        this->packet.clear();
        return img;
    }
    
    SerialResponseMessageType getMessageType() {
        return this->messageType;
    }
};
#endif