# rosmsg_compressor

Utility package to convert between ROS messages (standard or custom) and Compressed Raw Bytes.

## Sample Usage

Have a look at [sample code](src/sample_node.cpp).

To convert from any ROS message type to compressed raw bytes, 

```
void callback(const T& msg)
{
    ROS_INFO("Received msg..");

    // rosmsg -> bytes
    std::vector<uint8_t> buffer;
    RosMsgCompressor::serialize_to_byte_array(msg, buffer);
```

To convert from compressed raw bytes to the ROS message type,

```
    // bytes -> rosmsg
    T deserialized_msg;
    RosMsgCompressor::deserialize_from_byte_array(buffer, deserialized_msg);


    ROS_INFO("Published msg..");
    pub.publish(deserialized_msg);
```

## How does it work

It exploits the power of C++ templating and the fact that `ros::serialization`
applies to any ROS message, be it custom or standard. Hence, there is already a 
standard way to serialize and de-serialize the data. 

Compression is also performed internally to reduce packet size and similarly decompression 
is done automatically to recover the actual ROS message.