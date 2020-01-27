# Data Compressor

This package handles the compression of Data. Overall the following items are sent via telemetry:
    - LaserScan + Position + time
    - Artifact detections + Position + time
    - Current status

Additionally the following items are sent via LoRA from the base station
    * Start
    * End

# Overall Network Architecture
Packets must be less than 240bytes. Each

# Nodes
Two nodes are provided `decompressor_node` and `compressor_node`. `compressor_node` runs on the UGV and `decompressor_node` runs on the

# Launch files