from utils import utils
from pydoc import locate
import time

# all object detectors must implement 3 functions, __init__(optional config_file), detect(frame) and get_inference_time()

class ObjectDetector:
    def __init__(self):
        config = utils.get_config('object_detector_config.yaml')
        module_name = config['object_detector_module']
        print("Module Name: " + str(module_name))
        ObjectDetectorClass = locate(module_name)

        if 'object_detector_config' in config:
            object_detector_config = config['object_detector_config']
            self.object_detector = ObjectDetectorClass(object_detector_config)
        else:
            self.object_detector = ObjectDetectorClass()

        self.inference_time = -1

    # frame is opencv mat frame, return list of DetectedObject
    def detect(self, frame):
        start = time.time()
        detected_list = self.object_detector.detect(frame)
        self.inference_time = time.time() - start

        return detected_list

    # returns inference time in seconds. time <= 0 implies detector not running
    def get_inference_time(self):
        return self.inference_time