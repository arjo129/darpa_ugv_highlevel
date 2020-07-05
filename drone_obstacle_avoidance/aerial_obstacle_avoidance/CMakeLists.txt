cmake_minimum_required(VERSION 2.8.3)
project(aerial_obstacle_avoidance)

find_package(catkin REQUIRED roscpp tf octomap_msgs)

find_package(octomap REQUIRED)

catkin_package(
   DEPENDS octomap
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)


add_executable(apf_planner 
    include/dmath/geometry.h
    src/apf_planner.cpp
)

target_link_libraries(apf_planner
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)