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
ZLIB compression. Once the packet is uncompressed, the 

# Nodes
Two nodes are provided `decompressor_node` and `compressor_node`. `compressor_node` runs on the UGV and `decompressor_node` runs on the base station. 

## `decompressor_node` AKA the "Base Station"
For each robot, the decompressor node publishes the following topics:
- `/robot_n/scan` - A compressed scan. Note: Laserscan compression is lossy [sensor_msgs/LaserScan message]
- `/robot_n/wifi` - List of wifi Hotspots detected 
- `/robot_n/co2` - Current CO2 reading
- `/robot_n/vents` - Thermal Vents
- `/robot_n/manakin` - Manakin detections
- `/robot_n/odom` - Robot Odometry [nav_msgs/Odom message]
It also subscribes to the following:
- `/robot_n/estop`

# Launch files

# Relevant files