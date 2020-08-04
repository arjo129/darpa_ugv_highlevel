# exploration_goal

ROS package that takes in local frontier graphs (of type `graph_msgs`) and merge them into an internal global
frontier graph. It ensures that previously discovered local frontier points received are not duplicated on the 
main global graph. The next goal to traverse will then be published by the package to the `ugv_obstacle_avoidance`
package. 

No frontier detection is done by the package, only graph state maintainence and next exploration goal selection.
The global graph can be published as a `graph_msgs` with visualization options in rviz for easier debugging.


## TODO
- Use BGL Data Structure
    - Figure out subgraph addition / merging

- KDtree
    - use radius check for duplicate points 

- specify how to merge graph, like local graph with wrong index, then check against kd-tree,
    then find repeated vertex and merge new ones to global graph
- Goal selection algo (can be simple)
- Check when robot has stopped moving to give next goal