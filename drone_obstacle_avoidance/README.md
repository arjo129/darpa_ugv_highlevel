## Aerial Obstacle Avoidance

### Description
- This package uses the simulation provided by subt repo 
- The objective is to avoid obstacle while manning a drone
- Algorithm does the following
   - Set the drone airbone
   - Subscribe to the position from clicked point
   - Convert point clouds to octomap
   - Find the min distance to an obstacle on octomap
   - Calculate the velocity vector accounting for the obstacle
   - Publish the twist message with the new velocity vector (Note that if the drone move too fast the twist message may not be published in time to stop the collision)

### Setup
```
sudo apt install ros-melodic-octomap-*
```

### Launch
- Terminal 1
```
./subt/docker/run.bash osrf/subt-virtual-testbed:latest cave_circuit.ign worldName:=simple_cave_01 robotName1:=MARBLE_QAV500 robotConfig1:=MARBLE_QAV500_SENSOR_CONFIG_1
```
- Terminal 2
```
roslaunch aerial_obstacle_avoidance aerial_obstacle_avoidance.launch
```
- Terminal 3
```
launch teleop_twist_keyboard teleop_twist_keyboard.launch
```

Resources
- https://github.com/osrf/subt
- https://github.com/ros-teleop/teleop_twist_keyboard
- https://github.com/DaikiMaekawa/quadrotor_moveit_nav