## Dashboard

### Install 
- Build package
```
catkin_make
```
- Setup testing score server with https://bitbucket.org/subtchallenge/test_scoring_server/src/master/

### Run
- Run testing score server
```
docker-compose up --build
```
- Start service node
```
rosrun interface_protocol interface_server.py
```
- Start rviz dashboard
```
rviz -d ./src/darpa_ugv_highlevel/darpa_dashboard_rviz/rviz/config.rviz
```

### Resources
- https://stackoverflow.com/questions/21150890/qt-5-assign-slot-with-parameters-to-a-qpushbutton
- https://github.com/ros-visualization/visualization_tutorials
