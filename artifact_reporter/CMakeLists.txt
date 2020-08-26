cmake_minimum_required(VERSION 2.8.3)
project(artifact_reporter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2_msgs
  object_detection
  subt_msgs
  geometry_msgs
  subt_communication_broker
  subt_ign
)

find_package(Boost REQUIRED)

set(IGN_MATH_VER 6)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)
set(IGN_MSGS_VER 4)
find_package(ignition-msgs${IGN_MSGS_VER} REQUIRED)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(artifact_report_spam src/report_artifact_spam.cpp)
add_dependencies(artifact_report_spam ${catkin_EXPORTED_TARGETS})

target_link_libraries(artifact_report_spam
  ${catkin_LIBRARIES}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
)


