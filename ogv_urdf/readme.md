# ogv_urdf

Used to publish static TF tree of the OGV, including lidar_frame, realsense_frame, thermal_frame and floor frame (static transform to ground plane to get starting DARPA coordinate from AprilTag). 

To run, 

`roslaunch ogv_urdf ogv_urdf.launch`

Complete Static TF Tree for reference:

![OGV TF Tree](ogv_tf_tree.png)

Visual of URDF:

![OGV URDF](ogv_urdf_rviz.png)
