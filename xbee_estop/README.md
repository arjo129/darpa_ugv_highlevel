# Emergency Stop across XBee mesh
## Dependancies
This module uses the Python 3 xbee library: pip3 install digi-xbee

## estopRecieve.py
Running this program will start the Receiver server. It listens for any messages sent across the XBee mesh.
This server program is to be run on all TGV and OGV for safety purposes.

The server follows the behaviour as follows:

1) The server listens for anything sent across the mesh.
2) Upon receiving a message, it packages the message data and sends it to the **/e_stop** topic.

## estopSend.py
Running this program will immediately send a broadcast to a connected XBee Mesh device.
This script is to be run on the base station. 
Only execute this script to send the **E-Stop Signal** across the XBee Mesh.