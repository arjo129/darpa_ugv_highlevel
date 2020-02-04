# Apriltag detection to get centre point
## Dependancies
This module requires the [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) stuff to be built and running.
The wiki page of apriltag_ros is [here](http://wiki.ros.org/apriltag_ros)

**NOTE**
apriltag_ros takes in the "image" and "camera_info" from the camera.
These can be changed in the launch file of **continuous_detection.launch**.

It publishes the tf transforms, tag detections information and a camera view of what is detected. 

You will need to inform the apriltag_ros module about the tag number and size.
This is done in the configuration file **tags.yml**.
Other settings that can be used to help detection is found in **settings.yml**.

## General
The module listens to "/tag_detections" published by the **apriltag_ros** package
If there are 2 detections, it gets the centre coordinate by averaging the x,y,z values of the position.

From the "camera_color_optical_frame", publishes to the TF tree the centre coordinate "middle".
Then a lookupTransform is performed to determine the vector between the base_link and the centre coordinate.
Since the "map" frame is generated on base_link, this vector is hence the same as the representation from "DARPA" frame to "map" frame.