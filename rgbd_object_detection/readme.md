# RGBD Object Detection

Simple RGBD based object detector that relies on traditional 2D deep learning detection and then 
looking up the 2D coordinate from the 3D pointcloud of the RGBD camera to give the local XYZ coordinates
of the **center** of the object. May face issues with very thin objects or those with a hole in the center.


## How to run

`roslaunch object_detection object_detection.launch`

Objects detected will be published as a custom message to `/object_detector/detected` of type `object_detection::DetectedObjectsMsg`.
Debug detection image feed is published to `/object_detector/debug/image_raw`.


# TODO
- Fix High Latency (Partly due to yolo-tiny and partly due to difficulty in synchronizing pointcloud and rgb data)
- No global coordinate provided yet
- No transfer learning done (yet) with images from simulator (refer to [robotika image set](https://github.com/robotika/subt-artf))