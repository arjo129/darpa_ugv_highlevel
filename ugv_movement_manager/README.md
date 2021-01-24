# UGV_movement_manager

UGV Movement_Manager package is a obstacle avoidance package to controls movement of the robot to the specified point while avoiding obstacles. 



|        Data required by package       |                 Description                                      |
|:---------------------------:|:-------------------------------------------------------------------------------------:|
|         Goal        |  The goal that the robot needs to go to. (geometry_msgs::PointStamped)  |
|        Traversable pointcloud input        | The traversable boundary pointcloud to be obtained from ugv_groundmap package (pcl::PointCloud<pcl::PointXYZ>) |
|        Robot's position and pose        |                        The current global position and pose of the robot (nav_msgs::Odometry)                             |


### Command to run package

```
roslaunch ugv_groundmap ugv_groundmap.launch
```

### Command to trigger Trivial Strategy

Robot can also be triggered to move using a more trivial strategy. In a separate terminal publish an empty msg to the toggle_strategy topic using the following command to toggle to and fro the normal exploration strategy and trivial strategy.

```
rostopic pub /toggle_strategy std_msgs/Empty "{}
```
