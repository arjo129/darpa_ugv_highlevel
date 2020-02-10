# Wifi sensor for ROS
## Dependancies
This module requires iwlist, nmcli and its tools to be installed
Fill in the **wifi_device** in the wifi_sensor_node.py depending on the device used on as shown in ifconfig

**NOTE** Usually sudo is needed to list access points other than the one that it is connected to. A work around was found which is to use **nmcli** to scan before calling on iwlist.

## General
The module runs the iwlist bash command using the **subprocess** library (The **os** library is set to be deprecated)
After receiving the output, it packages the following elements into custom messages: wifi.msg , wifiArray.msg

wifi.msg
 - std_msgs/String ssid
 - std_msgs/String signal
 - std_msgs/String quality
 - std_msgs/String last_beacon
 - std_msgs/String mac_address
 - nav_msgs/Odometry odom

wifiArray.msg
 - int32 count
 - wifi_sensor/wifi[] data

Finally publishing the data to the **/wifi_ap** topic