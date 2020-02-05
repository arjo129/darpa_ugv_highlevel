# tgv_urdf

Used to publish static TF tree of the tgv, including lidar_frame, realsense_frame, thermal_frame and floor frame (static transform to ground plane to get starting DARPA coordinate from AprilTag). 

To run, 

`roslaunch tgv_urdf tgv_urdf.launch`

Complete Static TF Tree for reference:

![tgv TF Tree](tgv_tf_tree.png)

Visual of URDF:

![tgv URDF](tgv_urdf_rviz.png)
