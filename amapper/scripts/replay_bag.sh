#!/bin/bash
rosparam set use_sim_time true

rosbag play -l $1 --clock  --topics /asv/ahrs8/data /asv/nav/rpy /asv/puck/scan /asv/puck/velodyne_points /asv/quantum_radar/scan /asv/sbg/gps_pos /asv/sbg/gps_vel /asv/sick/cloud /asv/ukf_pose /asv/utm /clock /tf /tf_static

