#include <data_compressor/datapacket.h>

using namespace data_compressor;


std::vector<wireless_msgs::LoraPacket> PacketSendQueue::send(ByteStream bs, std::string to) {
    std::vector<PhysicalChunk> chunks = toPhysicalChunks(bs);
    std::vector<wireless_msgs::LoraPacket> packets;
    for(PhysicalChunk chunk: chunks){
        chunk.packetId = packetCounter;
        wireless_msgs::LoraPacket packet = chunk.toLoRa(to);
        packet.from.data = this->localAddress;
        packets.push_back(packet);
    }
    packetCounter++;
    return packets;
}