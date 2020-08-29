#include "Node.hpp"
#include <iostream>

namespace aodv
{
    Node::Node() :
        id(), addr(), broadcastAddr()
    {
    }

    Node::Node(uint32_t id, std::string addr, std::string broadcastAddr) :
        id(id), addr(addr), broadcastAddr(broadcastAddr)
    {
    }

    void Node::send(Eth &eth, subt::CommsClient* commsClient, bool isOriginating)
    {
        if (isOriginating) {
            // Overwrite seq and src, because this node is originating eth.
            eth.seq = this->seq;
            eth.src = this->addr;
            this->seq++;
        }
        
        const uint16_t inputPayloadLength = eth.payloadLength;

        // Where seg is a segment, aodv::MAX_MESSAGE_SIZE/2 == aodv::ETH_NONVAR_LEN + seg.srcLength + seg.dstLength + seg.payloadLength
        // Where eth is a packet, there are sizeof(typeof(eth.segSeqMax)) segments.
        // So the maximum eth.payloadLength must be:
        const uint16_t maxPayloadLength = aodv::MAX_MESSAGE_SIZE - (aodv::ETH_NONVAR_LEN + eth.srcLength + eth.dstLength);
        if (eth.payloadLength < maxPayloadLength) 
        {
            eth.segSeqMax = 1;
        } 
        else 
        {
            eth.segSeqMax = (eth.payloadLength / maxPayloadLength) + ((eth.payloadLength % maxPayloadLength) != 0);
        }
        uint32_t maxSegments = eth.segSeqMax;

        // Split packet into segments.
        int pos = 0;
        for (int segSeq = 0; segSeq < maxSegments; segSeq++)
        {
            aodv::Eth seg(eth);
            seg.segSeq = segSeq;

            // Figure out the length of this segment.
            if (maxSegments == 1) // If we only have one segment, just use the input length.
            {
                seg.payloadLength = inputPayloadLength;
            }
            else if (segSeq == maxSegments - 1) // This is the last segment.
            {
                seg.payloadLength = inputPayloadLength - (maxPayloadLength * (maxSegments - 1));
            }
            else
            {
                seg.payloadLength = maxPayloadLength;
            }

            // Set the payload for this segment.
            seg.payload = eth.payload.substr(pos, seg.payloadLength);
            pos += seg.payloadLength;

            // Send segment.
            uint16_t segLen = aodv::ETH_NONVAR_LEN + seg.srcLength + seg.dstLength + seg.payloadLength;
            uint8_t msg[segLen] = { 0 };
            seg.serialise(msg);
            commsClient->SendTo(this->uint8_to_string(msg, segLen), this->broadcastAddr);

            /* 
                The subt data rate cap is 6750 bytes per second.
                By utilising the max 1500 bytes per message limit,
                this leads to a cap of 4.5 messages per second.

                Sleep for 250ms:
                250ms per message
                1s / 250ms = 4 messages per second

                4 < 4.5

                So long we keep to less than 4.5 messages per second, we should be fine.
            */
            uint64_t slp = 250000000;
            uint64_t start = ros::Time::now().toNSec();
            while (ros::Time::now().toNSec() - start < slp)
            {
                ros::spinOnce(); // To continue processing ros message callbacks
            }
        }
    }

    tl::optional<aodv::Eth> Node::receive(std::string data, subt::CommsClient* commsClient)
    {
        aodv::Eth seg = aodv::Eth();
        uint8_t msg[data.length()];
        this->string_to_uint8(msg, data);
        seg.deserialise(msg);

        if (seg.src == this->addr) {
            return tl::nullopt;
        }

        if (seg.dst == this->addr) {
            auto search = this->tableDesegment.find(seg.seq);
            if (search == tableDesegment.end()) {
                if (seg.segSeqMax == 1) {
                    return seg; // return optional object that contains seg.
                } else {
                    // No segments from this sequence number exist in tableDesegment.
                    tableDesegment[seg.seq] = std::vector<aodv::Eth>(seg.segSeqMax, aodv::Eth());
                    tableDesegment[seg.seq][seg.segSeq] = seg;
                }

            } else {
                tableDesegment[seg.seq][seg.segSeq] = seg;
                bool flag = true;
                for (aodv::Eth seg : tableDesegment[seg.seq]) {
                    if (seg == aodv::Eth()) {
                        flag = false;
                    }
                }

                if (flag) {
                    // All segments have been received, perform desegmentation.
                    // aodv::Eth seg;
                    uint64_t payloadLengthTotal = 0;
                    for (aodv::Eth seg : tableDesegment[seg.seq]) {
                        payloadLengthTotal += seg.payloadLength;
                    }

                    // Copy all the segments payload into one final payload according to order.
                    // uint8_t* payload = (uint8_t*)malloc(payloadLengthTotal);
                    // memcpy(payload, tableDesegment[seg.seq][0].payload, tableDesegment[seg.seq][0].payloadLength);
                    // for (uint32_t i=1; i<seg.segSeqMax; i++) {
                    //     memcpy(payload + tableDesegment[seg.seq][i-1].payloadLength, tableDesegment[seg.seq][i].payload, tableDesegment[seg.seq][i].payloadLength);
                    // }

                    std::string payload = "";
                    payload.reserve(payloadLengthTotal);
                    for (int i = 0; i < seg.segSeqMax; i++)
                        {
                            payload += tableDesegment[seg.seq][i].payload;
                        }

                    // Packet must consist of at least one segment. Copy fields of first segment.
                    aodv::Eth eth = aodv::Eth(tableDesegment[seg.seq][0]);
                    eth.payloadLength = payloadLengthTotal;
                    eth.payload = payload;
                    return eth; // return optional object that contains eth.
                }

            }

        } else {
            auto search = this->tableAddr.find(seg.src);
            if (search == this->tableAddr.end()) {
                // No segments from this address exist in tableAddr.
                this->tableAddr[seg.src] = std::unordered_map<uint32_t, std::vector<bool>>();
                this->tableAddr[seg.src][seg.seq] = std::vector<bool>(seg.segSeqMax);
                this->tableAddr[seg.src][seg.seq][seg.segSeq] = true;
                this->send(seg, commsClient, false);

            } else {
                std::unordered_map<uint32_t, std::vector<bool>> tableSeq = search->second;
                auto search2 = tableSeq.find(seg.seq);
                if (search2 == tableSeq.end()) {
                    // No segments from this sequence number exist in tableSeq.
                    tableSeq[seg.seq] = std::vector<bool>(seg.segSeqMax);
                    tableSeq[seg.seq][seg.segSeq] = true;
                    this->send(seg, commsClient, false);

                } else {
                    if (!tableSeq[seg.seq][seg.segSeq]) {
                        // Segment was not already forwarded.
                        tableSeq[seg.seq][seg.segSeq] = true;
                        this->send(seg, commsClient, false);
                    }
                }
            }
        }

        return tl::nullopt; // return optional object that does not contain a value.
    }

    std::string Node::uint8_to_string(uint8_t b[], std::string::size_type l)
    {
        return std::string((char *)b, l);
    }

    void Node::string_to_uint8(uint8_t b[], std::string s)
    {
        for (int i = 0; i < s.length(); i++)
        {
            b[i] = s[i];
        }
    }
}
