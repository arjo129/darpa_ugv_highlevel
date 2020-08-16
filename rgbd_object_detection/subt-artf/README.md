# subt-artf
Dataset of Artifacts for SubT Challenge

Most of the YoloV3 transfer learning steps were taken from here:

    https://github.com/ultralytics/yolov3/wiki/Example:-Transfer-Learning


## How to use

First, generate the full annotations data in Darknet format from the `exported_annotations.json` from the robotika subt repo.

`cd annotation-tool && python generate_darknet.py ../labels/exported_annotations.json`

Next, type below to transfer learn over the coco dataset

`python3 ../yolov3/train.py --data labels/subt_caves.data --cfg labels/yolov3-subt-cave.cfg --weights ../yolov3/weights/yolov3-tiny.weights --transfer`