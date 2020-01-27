# Data Compressor

This package handles the compression of Data for transmission over our mesh network. 
Overall the following items are sent via telemetry:

- LaserScan + Position + time
- Artifact detections + Position + time [Not Yet Implemented]
- Current status [Not Yet Implemented]
- Repeater Node Drop points [Not Yet Implemented]

Additionally the following items are sent via LoRA from the base station

* Start
* End

# Overall Network Architecture
Packets must be less than 240bytes. Each packet is represented by a `wireless_msgs/LoraPacket` message. The data field in the packet is encoded using
ZLIB compression. Once the packet is uncompressed, the first byte refers to the message type. These can be found in `include/data_compressor/protocol.h`.
Individual messages are encoded and parsed in their own manner.

# Nodes
Two nodes are provided `decompressor_node` and `compressor_node`. `compressor_node` runs on the UGV and `decompressor_node` runs on the base station. 

## `decompressor_node` AKA the "Base Station"
For each robot, the decompressor node publishes the following topics:
- `/robot_n/scan` - A compressed scan. Note: Laserscan compression is lossy [`sensor_msgs/LaserScan` message]
- `/robot_n/wifi` - List of wifi Hotspots detected 
- `/robot_n/co2` - Current CO2 reading
- `/robot_n/vents` - Thermal Vents
- `/robot_n/manakin` - Manakin detections
- `/robot_n/status` - Status 
- `/robot_n/poop_trail` - The poop trail left by the dropped repeaters.
- `/robot_n/odom` - Robot Odometry [`nav_msgs/Odom` message]
It also subscribes to the following:
- `/robot_n/estop`
- `/robot_n/start`

## `compressor_node` 
This node runs on the robot and subscribes to the following topics:
- `/scan`
- `/co2`
- `/odom`
It publishes the following topics
- `/estop`
- `/start`

# Launch files
There are launch files provided for the UGV and base station respectively.

# Develope guide(ish)
A rough overview of the code here is given:
- `src/laserscan.cpp` and `include/data_compressor/msgs/laserscan.h` - Compression tactic for laserscan. Sample only every n ranges to get a rough idea of what the surrounding looks like. With the 360 points given by rplidar. Also convert all floats to uint16_t.
- `include/data_compressor/parser.h` and `src/parser/*` - Parser Utilities
- `reciever.cpp` - The `decompressor_node` AKA base station node
- `sender.cpp` - The `compressor_node` AKA what runs on each robot
Unit tests may be found in the `test` directory