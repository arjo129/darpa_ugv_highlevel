#include <data_compressor/data_compressor.h>

using namespace data_compressor;

DataPacket::DataPacket(const DataPacket& other) {
    this->estimated_pose = geometry_msgs::Pose(other.estimated_pose);
    this->packet_type = other.packet_type;
    this->uncompressed_size = other.uncompressed_size;
    this->data = std::vector<uint8_t>(other.data);
}

ByteStream DataPacket::serialize() {
    ByteStream bs;
    union {
        double value;
        char bytes[8];
    } double_converter;

    union {
        long value;
        char bytes[8];
    } long_converter;

    union {
        MessageType value;
        char bytes[2];
    } MessageType_converter;

    //Convert position data
    double_converter.value = this->estimated_pose.position.x;
    for(int i = 0 ; i < 8; i++)
        bs.data.push_back(double_converter.bytes[i]);
    double_converter.value = this->estimated_pose.position.y;
    for(int i = 0 ; i < 8; i++)
        bs.data.push_back(double_converter.bytes[i]);
    double_converter.value = this->estimated_pose.position.z;
    for(int i = 0 ; i < 8; i++)
        bs.data.push_back(double_converter.bytes[i]);

    //Convert orientation data
    double_converter.value = this->estimated_pose.orientation.x;
    for(int i = 0 ; i < 8; i++)
        bs.data.push_back(double_converter.bytes[i]);
    double_converter.value = this->estimated_pose.orientation.y;
    for(int i = 0 ; i < 8; i++)
        bs.data.push_back(double_converter.bytes[i]);
    double_converter.value = this->estimated_pose.orientation.z;
    for(int i = 0 ; i < 8; i++)
        bs.data.push_back(double_converter.bytes[i]);
    double_converter.value = this->estimated_pose.orientation.w;
    for(int i = 0 ; i < 8; i++)
        bs.data.push_back(double_converter.bytes[i]);

    //Convert message type
    MessageType_converter.value = this->packet_type;
    for(int i = 0 ; i < 2; i++)
        bs.data.push_back(MessageType_converter.bytes[i]);
    
    //Convert uncompressed size
    long_converter.value = this->uncompressed_size;
    for(int i = 0 ; i < 8; i++)
        bs.data.push_back(long_converter.bytes[i]);

    //Data
    for(int i = 0; i < this->data.size(); i++){
        bs.data.push_back(this->data[i]);
    }

    return bs;
}

DataPacket data_compressor::fromByteStream(ByteStream bs) {
    DataPacket dp;
    union {
        double value;
        char bytes[8];
    } double_converter;

    union {
        long value;
        char bytes[8];
    } long_converter;

    union {
        MessageType value;
        char bytes[2];
    } MessageType_converter;

    //Position
    int pointer = 0;
    for(int i = 0; i < 8; i++){
        double_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 8;
    dp.estimated_pose.position.x = double_converter.value;
     for(int i = 0; i < 8; i++){
        double_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 8;
    dp.estimated_pose.position.y = double_converter.value;
     for(int i = 0; i < 8; i++){
        double_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 8;
    dp.estimated_pose.position.z = double_converter.value;

    //Orientation
    for(int i = 0; i < 8; i++){
        double_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 8;
    dp.estimated_pose.orientation.x = double_converter.value;
    for(int i = 0; i < 8; i++){
        double_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 8;
    dp.estimated_pose.orientation.y = double_converter.value;
    for(int i = 0; i < 8; i++){
        double_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 8;
    dp.estimated_pose.orientation.z = double_converter.value;
    for(int i = 0; i < 8; i++){
        double_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 8;
    dp.estimated_pose.orientation.w = double_converter.value;

    //Message type
    for(int i = 0; i < 2; i++){
        MessageType_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 2;
    dp.packet_type = MessageType_converter.value;

    //Uncompressed size
    for(int i = 0; i < 8; i++){
        long_converter.bytes[i] = bs.data[pointer + i];
    }
    pointer += 8;
    dp.uncompressed_size = long_converter.value;

    for(;pointer < bs.data.size(); pointer++) {
        dp.data.push_back(bs.data[pointer]);
    }

    return dp;
}

bool DataPacket::operator==(const DataPacket& other) {
    if(this->estimated_pose.position.x != other.estimated_pose.position.x) {
        return false;
    }
    if(this->estimated_pose.position.y != other.estimated_pose.position.y) {
        return false;
    }
    if(this->estimated_pose.position.z != other.estimated_pose.position.z) {
        return false;
    }
    if(this->estimated_pose.orientation.x != other.estimated_pose.orientation.x) {
        return false;
    }
    if(this->estimated_pose.orientation.y != other.estimated_pose.orientation.y) {
        return false;
    }
    if(this->estimated_pose.orientation.z != other.estimated_pose.orientation.z) {
        return false;
    }
    if(this->estimated_pose.orientation.w != other.estimated_pose.orientation.w) {
        return false;
    }
    if(this->packet_type != other.packet_type) {
        return false;
    }
    if(this->uncompressed_size != other.uncompressed_size) {
        return false;
    }
    if(this->data != other.data) {
        return false;
    }
    return true;
}