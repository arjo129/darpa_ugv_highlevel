# RGBD Object Detection

Simple RGBD based object detector that relies on traditional 2D deep learning detection and then 
looking up the 2D coordinate from the 3D pointcloud of the RGBD camera to give the local XYZ coordinates
of the **center** of the object. May face issues with very thin objects or those with a hole in the center.


## How to run

`roslaunch object_detection object_detection.launch`

Objects detected will be published as a custom message to `/object_detector/detected` of type `object_detection::DetectedObjectsMsg`.
Debug detection image feed is published to `/object_detector/debug/image_raw`.


## YOLO Transfer Learning

Transfer learning setup is done using the `subt_artf` folder (originally from robotika subt team). Basic scripts written to convert to yolo format and to train the model. Further docs in the `subt_artf` readme.


## TODO
- Fix High Latency (Partly due to yolo-tiny and partly due to difficulty in synchronizing pointcloud and rgb data)
- No global coordinate provided yet
- No 
