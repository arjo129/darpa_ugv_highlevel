cmake_minimum_required(VERSION 3.0.2)
project(noroute_mesh)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  subt_ign
  message_generation
  subt_communication_broker
  rospy_message_converter
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    neighb.msg
    neighbArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  send_map.srv
  neighbour.srv
  send_artifact.srv
  string_to_map.srv
  map_to_string.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
   CATKIN_DEPENDS message_generation message_runtime std_msgs nav_msgs
)

include_directories(
  include
  eth
  lib
  node
  utilities
  ${catkin_INCLUDE_DIRS}
)

add_library(eth eth/Eth.cpp eth/Eth.hpp)
add_library(crc lib/crc32.c)
add_library(node node/Node.cpp node/Node.hpp)
add_library(serialisers utilities/serialisers.cpp utilities/serialisers.hpp)

add_executable(noroute_mesh_node src/main.cpp)

target_link_libraries(eth
  serialisers
)

target_link_libraries(node
  eth
  subt_communication_broker
)

target_link_libraries(noroute_mesh_node
  eth
  crc
  node
  serialisers
  ${catkin_LIBRARIES}
)
