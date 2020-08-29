#ifndef NODE_H_
#define NODE_H_

#include "../eth/Eth.hpp"
#include "../include/tl/optional.hpp"
#include <stdint.h>
#include <cassert>
#include <queue>
#include <string>
#include <unordered_map>
#include "subt_communication_broker/subt_communication_client.h"
#include <tuple>
#include <vector>
#include <cstring>

namespace aodv
{
    // Assume send_link has maximum message size of 1500 octets.
    const uint16_t MAX_MESSAGE_SIZE = 1500;

    class Node
    {
    private:
        /**
         * @brief Sequence number.
         * 
         */
        uint32_t seq = 0;
        
        /**
         * @brief RREQ identifier.
         * 
         */
        uint32_t id = 0;

        /**
         * @brief address.
         * 
         */
        std::string addr;

        /**
        * @brief Address on which broadcasts are made.
        * 
        */
        std::string broadcastAddr;

        /**
         * @brief Table of addresses, key: address, value: tableSeq.
         * tableSeq, key: sequence number, value: vector of (index: segment sequence numbers, value: bool).
         *
         * Tracks which segments are forwarded by this node.
         * 
         */
        std::unordered_map<std::string, std::unordered_map<uint32_t, std::vector<bool>>> tableAddr;

        /**
         * @brief Table storing segments for desegmentation into packets, key: sequence number, value: vector of segments.
         * 
         */
        std::unordered_map<uint32_t, std::vector<aodv::Eth>> tableDesegment;

    public:
        /**
         * @brief Construct a new object with all members zero initialised.
         * 
         */
        Node();
  
        /**
         * @brief Construct a new object.
         * 
         */
        Node(uint32_t id, std::string addr, std::string broadcastAddr);

        /**
         * @brief Send a control packet or a data packet with this->seq and this->addr.
         *
         * @param eth ethernet packet. No limit on eth.payloadLength (other than sizeof(eth.payloadLength).
         * @param send_link method that sends on link level.
         */
        void send(Eth &eth, subt::CommsClient* commsClient, bool isOriginating);

        /**
         * @brief Receive a control packet or a data packet.
         *
         * @param receive_link method that receives on link level.
         */
        tl::optional<aodv::Eth> receive(std::string data, subt::CommsClient* commsClient);
  
        /**
         * @brief Represent an arbitrary uint8_t buffer as a string.
         * 
         * @param b uint8_t buffer.
         * @param l length of uint8_t buffer.
         * @return string representation.
         */
        std::string uint8_to_string(uint8_t b[], std::string::size_type l);
  
        /**
         * @brief Parse represented string as an arbitrary uint8_t buffer.
         * Inverse of uint8_to_string(uint8_t b[], std::string::size_type l).
         * 
         * @param b uint8_t buffer. Must have length a multiple of 4.
         * @param s string.
         */
        void string_to_uint8(uint8_t b[], std::string s);
    };
}

#endif // NODE_H_
