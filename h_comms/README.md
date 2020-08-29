# h_comms
High level communications for team NUS SEDS.

## Install these prerequisites for ROS message conversion
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg

## Start the simulator with 3 robots
source ~/subt_ws/install/setup.bash

ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1 robotName2:=X2 robotConfig2:=X1_SENSOR_CONFIG_1 robotName3:=X3 robotConfig3:=X1_SENSOR_CONFIG_1

## Using noroute_mesh package
Make sure the workspace has been sourced.

First you need to launch the ROS messages converter ROS service:
```
roslaunch noroute_mesh converter_service.launch
```

For each robot in simulation, you need to run a communication node for it:
```
roslaunch noroute_mesh comms_node.launch name:={robotName} id:={robotId} 
```
`{robotName}` is to be replaced (including '{ }') with the name of the robot. It should not contain spaces. The names should corresponds to the name used by subt simulation.

`{robotId}` is to be replaced (including '{ }') with an identifier number for the robot. It should not contain spaces or special characters(just digits), any number within the range [0, 4294967295]. This identifier must be unique and cannot be one that already exists.

Example:
```
roslaunch noroute_mesh comms_node.launch name:=X1 id:=1
```

### Sending maps
When a communication node gets started, a ROS service named `/{robotName}/send_map` will be advertised. The request message contains `string:dest` and `nav_msgs/OccupancyGrid:grid` fields.

`string:dest` should be filled with the robot name of the intended recipient.

`nav_msgs/OccupancyGrid:grid` should be filled with the OccupancyGrid map that is to be sent.

NOTE: At the moment, the response from calling this service has no meaning.

### Querying neighbours
When a communication node gets started, a ROS service named `/{robotName}/get_neighbour` will be advertised. The request message is an empty message.

In the response message, it contains `int32:num_neighbours` and `string:response` fields.

`int32:num_neighbours` tells you how many neighbours this robot can see.

`string:response` contains the information of each robot. Each robot has a `robotName` and `RSSI` value associated with it, both values are delimited by a `,`. Multiple robots are delimited by a `|`. For example, a `string:response` field could look like this: `X1,-20|X2,-25|X3,-22.1`

### Sending Artifacts
When a communication node gets started, a ROS service named `/{robotName}/send_artifact` will be advertised. The request message contains `int32:x`, `int32:y`, `int32:z` and `string:type` field.

`string:type` type field will inform the judge of the type of artifact found. It can be of the following types:
- TYPE_BACKPACK
- TYPE_DUCT
- TYPE_DRILL
- TYPE_ELECTRICAL_BOX
- TYPE_EXTINGUISHER
- TYPE_PHONE
- TYPE_RADIO
- TYPE_RESCUE_RANDY
- TYPE_TOOLBOX
- TYPE_VALVE
- TYPE_VENT
- TYPE_GAS
- TYPE_HELMET
- TYPE_ROPE

In the response message, it contains `bool:response` which indicates whether the report was sent successfully.