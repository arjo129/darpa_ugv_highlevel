#include <data_compressor/datapacket.h>

using namespace data_compressor;

PacketRecieveQueue::PacketRecieveQueue() {

}

std::vector<ByteStream> PacketRecieveQueue::getBuffer() {
    std::vector<ByteStream> bs;
    std::vector<long> toBeRemoved;
    for(auto packets: this->incomingPacketQueue) {
        if(!isPacketComplete(packets.second))
            continue;
        bs.push_back(reassemblePacket(packets.second));
        toBeRemoved.push_back(packets.first);
    }
    for(long packet: toBeRemoved){
        this->incomingPacketQueue.erase(packet);
    }
    return bs;
}

void PacketRecieveQueue::enqueuePacket(PhysicalChunk packet, int arrivalTime) {

    this->incomingPacketQueue[packet.packetId].insert(packet);
    this->packetTimeofArrival.push(std::pair<long,long>(-arrivalTime, packet.packetId));
}

void PacketRecieveQueue::garbageCollect(int currentTime) {

    while(true) {
        std::pair<long,long> packetRecvd = this->packetTimeofArrival.top();
        if((currentTime - (-packetRecvd.first)) < MAX_TIME_TO_HOLD_PACKET)
            break;
        this->packetTimeofArrival.pop();
        this->incomingPacketQueue.erase(packetRecvd.second);
    }
}