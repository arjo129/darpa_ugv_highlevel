cmake_minimum_required(VERSION 2.8.3)
project(rosmsg_compressor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  graph_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(ZLIB)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs graph_msgs nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/sample_node.cpp)
add_executable(sample_node src/sample_node.cpp)
add_dependencies(sample_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( sample_node
  ${ZLIB_LIBRARIES}
  ${catkin_LIBRARIES}
)
