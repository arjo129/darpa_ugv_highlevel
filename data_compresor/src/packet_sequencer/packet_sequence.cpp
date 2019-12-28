#include <data_compressor/datapacket.h>

using namespace data_compressor;

bool PhysicalChunk::operator<(const PhysicalChunk& other) const{
    return this->sequenceNumber < other.sequenceNumber; 
}

PacketSequencer::PacketSequencer() {
}

std::vector<PhysicalChunk> data_compressor::toPhysicalChunks(ByteStream packet) {
    std::vector<PhysicalChunk> chunklist;
    int i  = 0, sequence_number = 0; 
    while(i < packet.data.size()) {
        PhysicalChunk p;
        while(p.data.size() < MAX_PACKET_SIZE && i < packet.data.size()) {
            p.data.push_back(packet.data[i]);
            i++;
        }
        p.sequenceNumber = sequence_number;
        sequence_number++;
        chunklist.push_back(p);
    }

    for(int i = 0; i < chunklist.size(); i++) {
        chunklist[i].maxCount = sequence_number;
    }
    return chunklist;
}



bool data_compressor::isPacketComplete(std::set<PhysicalChunk> fragments) {
    if(fragments.size() == 0)
        return false;
    if(fragments.size() == fragments.begin()->maxCount)
        return true;
    return false;
}

ByteStream data_compressor::reassemblePacket(std::set<PhysicalChunk> fragments) {
    ByteStream packet;
    for(PhysicalChunk chunk: fragments){
        for(int i = 0; i < chunk.data.size(); i++){
            packet.data.push_back(chunk.data[i]);
        }
    }
    return packet;
}