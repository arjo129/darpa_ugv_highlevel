#include <cstring>
#include "Eth.hpp"
#include "../utilities/serialisers.hpp"
#include <iostream>

namespace aodv
{
    Eth::Eth() : 
        seq(), segSeq(), segSeqMax(), dstLength(), dst(), src(), srcLength(), payloadLength()
    {
    }

    Eth::Eth(uint32_t seq, uint32_t segSeq, uint32_t segSeqMax, uint16_t dstLength, std::string dst, uint16_t srcLength, std::string src, uint16_t payloadLength) :
        seq(seq), segSeq(segSeq), segSeqMax(segSeqMax), dstLength(dstLength), dst(dst), srcLength(srcLength), src(src), payloadLength(payloadLength)
    {
    }

    void Eth::serialise(uint8_t data[])
    {
        size_t i = 0;
        serialisers::copyU32(&data[i], seq);
        i += 4;
        serialisers::copyU32(&data[i], segSeq);
        i += 4;
        serialisers::copyU32(&data[i], segSeqMax);
        i += 4;
        serialisers::copyU16(&data[i], dstLength);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&dst[0]), dstLength);
        i += dstLength;
        serialisers::copyU16(&data[i], srcLength);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&src[0]), srcLength);
        i += srcLength;
        serialisers::copyU16(&data[i], payloadLength);
        i += 2;
        memcpy(&data[i], reinterpret_cast<const uint8_t*>(&payload[0]), payloadLength);
        i += payloadLength;
        serialisers::copyU32(&data[i], crc);
    } // TODO calculate crc
    
    void Eth::deserialise(uint8_t data[])
    {
        size_t i = 0;
        seq = serialisers::getU32(&data[i]);
        i += 4;
        segSeq = serialisers::getU32(&data[i]);
        i += 4;
        segSeqMax = serialisers::getU32(&data[i]);
        i += 4;
        dstLength = serialisers::getU16(&data[i]);
        i += 2;
        dst = std::string(dstLength, 1);
        for (int j=0; j<dstLength; j++) {
            dst[j] = data[i+j];
        }
        i += dstLength;
        srcLength = serialisers::getU16(&data[i]);
        i += 2;
        src = std::string(srcLength, 1);
        for (int j=0; j<srcLength; j++) {
            src[j] = data[i+j];
        }
        i += srcLength;
        payloadLength = serialisers::getU16(&data[i]);
        i += 2;
        payload = std::string(payloadLength, 1);
        for (int j=0; j<payloadLength; j++) {
            payload[j] = data[i+j];
        }
        i += payloadLength;
        crc = serialisers::getU32(&data[i]);
    }

    bool check(uint32_t crc)
    {
        return true;
    } // TODO calculate and check crc

    bool Eth::operator==(const aodv::Eth& eth)
    {
        return this->seq==eth.seq && this->segSeq==eth.segSeq && this->segSeqMax==eth.segSeqMax && this->dstLength==eth.dstLength && this->dst==eth.dst && this->src==eth.src && this->srcLength==eth.srcLength && this->payloadLength==eth.payloadLength && this->payload==eth.payload;
    }
}
