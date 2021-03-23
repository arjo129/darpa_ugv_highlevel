#ifndef _ROSMSG_SER_H
#define _ROSMSG_SER_H

#include <ros/ros.h>
#include <sstream>
#include <rosmsg_compressor/zip.h>

/*
    Helper functions to convert between ROS messages and compressed raw byte streams 
    for lower level networking between robots. 

    Note: ANY ROS message is supported, even custom ROS messages. Works due to C++ templates and ros::serialization

    http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes#Deserializing_from_Memory
    https://gist.github.com/facontidavide/2e9c198bdd806f4bea32c1335cc3d020
*/


namespace RosMsgCompressor
{
    /**
     * @brief Convert from ROS message of type *T* to compressed raw bytes
     * 
     * @tparam T Type of ROS message. Eg: geometry_msgs::PointStamped
     * @param msg ROS message object to convert to compressed raw bytes
     * @param destination_buffer Destination reference of the converted ROS message
     */
    template <typename T>
    void serialize_to_byte_array(const T& msg, std::vector<uint8_t>& destination_buffer)
    {
        // Allocate space on buffer
        const uint32_t length = ros::serialization::serializationLength(msg);
        destination_buffer.resize( length );

        // Convert rosmsg to raw bytes
        ros::serialization::OStream stream(destination_buffer.data(), length);
        ros::serialization::Serializer<T>::write(stream, msg);

        // Compress raw bytes
        destination_buffer = RosMsgCompressor::compressZip(destination_buffer);
    }
    
    /**
     * @brief Convert from compressed raw bytes to ROS message of type *T*
     * 
     * @tparam T Type of ROS message. Eg: geometry_msgs::PointStamped
     * @param source_buffer Source reference of the input compressed raw bytes
     * @param msg Converted ROS message reference output
     */
    template <typename T>
    void deserialize_from_byte_array(std::vector<uint8_t>& source_buffer, T& msg)
    {
        // Uncompress raw bytes
        source_buffer = RosMsgCompressor::uncompressZip(source_buffer);

        // Convert uncompressed raw bytes to rosmsg
        ros::serialization::IStream stream(source_buffer.data(), source_buffer.size());
        ros::serialization::Serializer<T>::read(stream, msg);
    }

}

#endif /* _ROSMSG_SER_H */
