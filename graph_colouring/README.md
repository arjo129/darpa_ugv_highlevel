Launch simulator with ground truth enabled
ign launch -v 4 competition.ign worldName:=tunnel_circuit_practice_01 circuit:=cave enableGroundTruth:=true robotName1:=X1 robotConfig1:=COSTAR_HUSKY_SENSOR_CONFIG_1

Run graph colouring -- independant from dots node for now
rosrun graph_colouring graph_colouring_node

Library functions of image geometry
https://docs.ros.org/en/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html