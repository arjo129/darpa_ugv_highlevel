# Interact with the command station scoring and mapping server
## Dependancies
This module requires the following ROS package:

[rospy_message_converter](https://github.com/uos/rospy_message_converter)
The module is used to convert the various ROS messages to JSON formats. This is needed for the mapping updates that will be sent to the command post.

## General
The server script requires the URL and the token to successfully send the message to the command post.
**URL**: The server to interact with for mapping and scoring. In the event that mapping and scoring servers are seperate, I have split them into seperate classes.

**token**: The server uses an authorization method of sending a **Bearer token** together with the request. This module just requires the location of the file containing the token (e.g /path/to/token/token.txt). The file should only contain the token and nothing else.

**Mapping Server**
Required changes:
Subscribed ROS topics are to be updated for the correct information to be sent to the command post.
sub_grid: subscribes to the map for the 2D OccupancyGrid
sub_pose: subscribes to the pose of the vehicles for the PoseArray
sub_mark: subscribes to the topic which publishes out markers?

Client scripts can adopt the samples which are included in the **/test** folder. It simply calls the services which are setup by running the interface_server.py script


| SERVICES              | REQUEST                                         | RESPONSE                     |
| --------------------- | ----------------------------------------------- | ---------------------------- |
| get_status            | NIL                                             | current run status           |
| post_report           | X,Y,Z,TYPE (e.g Backpack)                       | report about score changes   |
| post_mapping_update   | NIL (takes current OccupancyGrid from rostopic) | HTTP request success/failure |
| post_telemetry_update | NIL (takes current PoseArray     from rostopic) | HTTP request success/failure |
| post_markers_update   | NIL (takes current MarkersArray  from rostopic) | HTTP request success/failure |