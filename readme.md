# UGV + Base Station software
[![Build Status](https://travis-ci.com/arjo129/darpa_ugv_highlevel.svg?token=kEQyTbP2NRSN6ymJo7RV&branch=master)](https://travis-ci.com/arjo129/darpa_ugv_highlevel)
This is the software that runs on the UDOOs and on our computers. For firmware that runs on the teensies/arduinos see [here](https://github.com/lekoook/darpasubt/tree/custom_proto).

# Requirements
You need to have ROS Melodic Morena installed on Ubuntu 18.04. For building this run the following:
```
  sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-melodic-desktop-full ros-melodic-rosserial ros-melodic-rosserial-arduino ros-melodic-apriltag-ros 
  sudo apt-get install -y ros-melodic-teleop-twist-keyboard ros-melodic-move-base libfftw3-dev libqt5charts5-dev
```
# Developer Guide
This is a rough overview of which folder does what. Some packages may contain readmes to make things clearer.
- `amapper` - This package contains a utility library useful for handling navigation stuff.
- `apriltag_coordinates` - This package is responsible for getting our baseline calibration to the `DARPA` world frame.
- `data_compressor` - The data compressor utility that enables us top transmit compressed telemetry back to the base station
- `interface_protocol` - Adds support for interfacing to the DARPA server
- `loam_velodyne` - Mapping using LIDAR
- `mission_planner` - Current GUI for commanding the robots.
- `rf2o_laser_odometry` - Laser scan based odometry.
- `teensy_bridge` - drivers for various peripherals
- `thermal_camera` - consists of thermal camera fusion
- `wireless_msgs` - messages used for communications
- `xbee-estop` - comply with DARPA e-stop regulations.

## I'm still lost
What are you working on? If you are working one the following these are the packages you want to take a look at:

* Networking: `wireless_msgs`, `data_compressor`, `teensy_bridge`
* Peripherals and drivers: `teensy_bridge`
* Integration with DARPA: `apriltag_coordinates`, `interface_protocol`, `xbee-estop`
* Obstacle avoidance, frontier exploration: `simple_obstacle_avoidance`, `amapper`
