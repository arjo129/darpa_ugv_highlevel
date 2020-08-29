# DetectedObject

geometry_msgs/PointStamped local_point   # local TF frame (eg: camera_link) 3D point. Always filled.
geometry_msgs/PointStamped global_point  # global TF frame (eg: world) 3D point. Reports (0,0,0) with empty stamp if not configured
float32 confidence                  # confidence of detection. 0-1
string class_name                   # class label of object. example: person
uint16[] bbox                       # 2D bbox: centre_x, centre_y, w, h